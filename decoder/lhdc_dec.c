#include "lhdc_dec.h"
#include "lhdc_dec_internal.h"
#include "lhdc_bit_reader.h"
#include "lhdc_imdct.h"
#include "lhdc_sns_synth.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_tables.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>   /* snprintf used in g_lhdc_trace diagnostics */
#if defined(LHDC_HOST_BUILD)
  #define ESP_LOGI(tag, ...) do { printf("[I]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
  #define ESP_LOGW(tag, ...) do { printf("[W]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
  #define ESP_LOGE(tag, ...) do { printf("[E]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
#else
  #include "esp_log.h"
  #include "esp_timer.h"
  #include "esp_attr.h"
#endif

/* Place the hot per-element decode loops in IRAM. At 96k (N=960) the decode is
 * close to the real-time budget, so flash-cache misses caused by the Bluetooth
 * controller's interrupts can push individual frames over budget and cause
 * underruns. Executing from IRAM removes that jitter. (Host build: no-op.) */
#if defined(LHDC_HOST_BUILD)
  #define LHDC_HOT
#else
  #define LHDC_HOT IRAM_ATTR
#endif

/* Optional stage profiler: times the per-channel decode stages (entropy, IMDCT,
 * remainder). The periodic log output stalls the decode task on the UART and
 * disturbs audio, so it is disabled by default; set to 1 for bring-up only. */
#define LHDCV5_DEC_PROFILE 0
#if defined(LHDC_HOST_BUILD)
  #define LHDC_NOW_US() 0LL
#else
  #define LHDC_NOW_US() esp_timer_get_time()
#endif

/* 96k output level. Must be 1.0: the IMDCT 1/M normalization already yields the
 * correct level, so a full-scale source decodes to full scale. A boost above 1.0
 * would push loud content past full scale, where LHDC_OUT_SAMPLE hard-clips it
 * into continuous buzzing. System/device volume covers any small level offset. */
#define LHDC_96K_GAIN 1.0f

/* One-shot decode trace: set to 1 by the caller at stream start; the decoder
 * logs the first frame's leading-section values, then self-clears. Diagnostic. */
volatile int g_lhdc_trace = 0;

/*
 * Frame scramble: the first 8 payload bytes are permuted (permutation selected
 * by payload[8]&1) and XOR-masked. Byte 8 onward is plaintext. The decoder
 * inverts this in place.
 */
static const uint8_t LHDC_XOR_MASK[8] = {
    0xFF, 0xE7, 0x7A, 0xB3, 0xDA, 0xE5, 0xCD, 0x73
};

/*
 * Shared IMDCT window cache. The Princen-Bradley window depends only on
 * mdct_size and is read-only after generation, so one copy is shared across all
 * decoder instances rather than stored per instance. Regenerated only when
 * mdct_size changes. Not re-entrant, but the A2DP sink decodes on a single task.
 *
 * For 480 the window is a const flash table (lhdc_get_window_const), so no RAM
 * copy is needed. Other sizes fall back to a lazily allocated buffer.
 */
static float   *s_imdct_window = NULL;
static int      s_imdct_window_size = 0;

static const float *lhdc_dec_get_window(int mdct_size)
{
#if defined(LHDC_HOST_BUILD)
    /* Host-only hook: load the synthesis window from a raw float32 file
     * (LHDC_WINFILE) so candidate windows can be tested without recompiling. */
    { const char *wf = getenv("LHDC_WINFILE");
      if (wf) {
          static float s_winfile[2048]; static int s_winfile_n = 0;
          if (s_winfile_n != mdct_size) {
              FILE *f = fopen(wf, "rb");
              if (f) { s_winfile_n = (int)fread(s_winfile, sizeof(float), (size_t)mdct_size, f); fclose(f); }
          }
          if (s_winfile_n == mdct_size) return s_winfile;
      } }
#endif
    const float *cw = lhdc_get_window_const(mdct_size);
    if (cw) {                                  /* flash window for this size -> zero RAM */
        /* This size uses the flash const window, so free any RAM window kept from
         * a previous higher rate (e.g. after a 96k->48k switch). */
        if (s_imdct_window) { free(s_imdct_window); s_imdct_window = NULL; s_imdct_window_size = 0; }
        return cw;
    }
    if (mdct_size != s_imdct_window_size && mdct_size <= LHDC_DEC_MAX_MDCT_SIZE) {
        /* Allocate exactly this rate's window. Reallocate on a size change. */
        if (s_imdct_window) { free(s_imdct_window); s_imdct_window = NULL; }
        s_imdct_window = (float *)malloc(sizeof(float) * (size_t)mdct_size);
        if (!s_imdct_window) { s_imdct_window_size = 0; return NULL; }
        lhdc_gen_kbd_window(s_imdct_window, mdct_size);
        s_imdct_window_size = mdct_size;
    }
    return s_imdct_window;
}
/* perm[k] = source byte index that lands at position k after descrambling. */
static const uint8_t LHDC_PERM[2][8] = {
    { 4, 0, 1, 5, 7, 3, 2, 6 },
    { 6, 3, 7, 0, 2, 1, 4, 5 },
};

/* Descramble one channel's leading 8 bytes in place. Each channel's header is
 * scrambled independently, so every channel section — not just the first — must
 * be descrambled before its bit reader runs. */
static void lhdc_descramble_inplace_sel(uint8_t *buf, size_t avail, int sel)
{
    if (avail < 9) return;
    uint8_t tmp[8];
    sel &= 1;
    const uint8_t *perm = LHDC_PERM[sel];
    for (int i = 0; i < 8; i++) tmp[perm[i]] = buf[i];
    for (int k = 0; k < 8; k++) buf[k] = tmp[k] ^ LHDC_XOR_MASK[k];
}

static void lhdc_descramble_inplace(uint8_t *buf, size_t avail)
{
    if (avail < 9) return;
    lhdc_descramble_inplace_sel(buf, avail, buf[8] & 1);
}

__attribute__((unused))
static void lhdc_descramble(const uint8_t *in, uint8_t *out, size_t len)
{
    if (len < 9) {
        memcpy(out, in, len);
        return;
    }
    int sel = in[8] & 1;
    const uint8_t *perm = LHDC_PERM[sel];
    for (int i = 0; i < 8; i++) {
        out[perm[i]] = in[i];
    }
    for (int k = 0; k < 8; k++) {
        out[k] ^= LHDC_XOR_MASK[k];
    }
    memcpy(out + 8, in + 8, len - 8);
}

/*
 * Parse one LHDC V5 frame.
 *
 * A frame is [u16 LE header][payload]:
 *   payload_len = (hdr & 0x3FF) * 2
 *   flags       = hdr >> 10   (=1 for 48k/5ms stereo)
 * There is no per-frame sync word or codec-parameter header; the stream format
 * (sample rate, channels, bit depth, frame duration) comes from the A2DP CIE
 * captured in dec->config. The payload is copied into dec->payload_buf, where
 * the per-channel bit reader later operates on it.
 *
 * Returns the number of input bytes consumed (header + payload) via *consumed.
 */
static lhdc_dec_ret_t lhdc_dec_parse_header(lhdc_decoder_t *dec,
                                            const uint8_t *in_data,
                                            size_t in_bytes,
                                            size_t *consumed)
{
    lhdc_dec_frame_header_t *hdr = &dec->header;

    memset(hdr, 0, sizeof(*hdr));
    *consumed = 0;

    if (in_bytes < 2) {
        return LHDC_DEC_NEED_MORE_DATA;
    }

    uint16_t fhdr = (uint16_t)(in_data[0] | (in_data[1] << 8));   /* LE */
    size_t payload_len = (size_t)(fhdr & 0x3FF) * 2;
    uint16_t flags = (uint16_t)(fhdr >> 10);

    if (payload_len == 0 || payload_len > LHDC_DEC_MAX_FRAME_BYTES) {
        return LHDC_DEC_BITSTREAM_ERROR;
    }
    if (in_bytes < 2 + payload_len) {
        return LHDC_DEC_NEED_MORE_DATA;
    }

    /* Stream params come from the configured CIE, not the frame. */
    hdr->sample_rate       = dec->config.sample_rate;
    hdr->channels          = dec->config.channels ? dec->config.channels : 2;
    hdr->bit_depth         = dec->config.bit_depth;
    hdr->frame_duration_ms = dec->config.frame_duration;
    hdr->version           = LHDC_DEC_VERSION_1;
    hdr->frame_flags       = flags;
    hdr->frame_bytes       = (uint16_t)payload_len;

    /*
     * New PCM samples emitted per frame = the MDCT hop = mdct_size/2 (the
     * overlap; see lhdc_dec_overlap_add). The emit count is derived from the
     * band config's MDCT size rather than the time-based count
     * (sample_rate*dur/1000): the two coincide at 48/96/192k, but at 44.1k the
     * time-based value is 220 instead of 240, which would drop samples and shift
     * pitch. Using mdct_size/2 keeps every rate at the correct playback speed.
     */
    const lhdc_band_cfg_desc_t *bcfg = lhdc_get_band_cfg(
        hdr->sample_rate, hdr->frame_duration_ms);
    hdr->band_cfg_idx = (uint8_t)bcfg->cfg_idx;
    hdr->samples_per_channel = (uint16_t)(bcfg->mdct_size / 2);

    /* Copy the raw payload into the scratch buffer. Each channel's 8-byte header
     * is descrambled later in decode_channel, so the descramble selector can be
     * retried per channel. */
    memcpy(dec->payload_buf, in_data + 2, payload_len);

    lhdc_bit_reader_init(&dec->bit_reader, dec->payload_buf, payload_len);

#if defined(LHDC_HOST_BUILD)
    extern volatile int g_lhdc_trace;
    if (g_lhdc_trace > 0) {
        printf("[RAWPAY] first 20 raw (pre-descramble) bytes:");
        for (int b = 0; b < 20; b++) printf(" %02x", in_data[2 + b]);
        printf("\n");
        printf("[PAYLOAD] len=%zu  byte:flag(msb) for 0,118..130:\n", payload_len);
        printf("  b0 msb=%d\n", (dec->payload_buf[0] >> 7) & 1);
        for (int b = 118; b <= 130 && b < (int)payload_len; b++)
            printf("  b%d=0x%02x msb=%d gain9=%d\n", b, dec->payload_buf[b],
                   (dec->payload_buf[b] >> 7) & 1,
                   ((dec->payload_buf[b] << 1) | (dec->payload_buf[b+1] >> 7)) & 0x1FF);
    }
#endif

    *consumed = 2 + payload_len;
    return LHDC_DEC_OK;
}

/*
 * Global-gain exponent e(gain_idx). The encoder quantizes the SNS-shaped
 * spectrum by q[k] = round(spectrum[k] * 2^-e(gidx)); the decoder dequantizes
 * linearly: spectrum[k] = q[k] * 2^e(gidx).
 *
 * Ported bit-exactly from the reference decoder's level table (Android lhdcv5
 * Rust, src/common/lhdc_level.rs `LevelTable::init`/`calc`). The gain table is a
 * piecewise-linear curve of gain_idx:
 *   offset_max = a - (a-c)/575 * (L-50)   (a,c depend on bit depth AND rate)
 *   start      = trunc(offset_max / D)     (D = 4 for 24-bit, 2.5 for 16-bit)
 *   jump       = (offset_max - start) / (503 - start)   [503 = OFFSET_MAX-1]
 *   e(idx):  idx <= start           : idx
 *            start < idx <= 504      : start + (idx-start)*jump
 *            idx > 504               : start + (504-start)*jump + (idx-504)
 *            (clamped to 30)         [OFFSET_MAX = 504]
 *
 * The per-rate (a,c): 24-bit 44.1/48k (26.2,16.4); 96k (25.7,20.75);
 * 192k (25.7,24.42). 16-bit 44.1k (26.15,16.42); 48k (26.18,16.32).
 *
 * NOTE: an earlier port used the 48k constants for every rate and force-set
 * slope=0.0378, which happened to track the *phone* encoder's gain approximately
 * but is wrong for the true 96k/192k curves -- with a correct encoder the 192k
 * dequant scale is badly off. This exact per-rate table is required.
 */
static float lhdc_dec_gain_exponent(int gain_idx, int enc_frame_len, int bit_depth,
                                    uint32_t sample_rate)
{
    int L = enc_frame_len;
#if defined(LHDC_HOST_BUILD)
    { const char *ld = getenv("LHDC_LDIV"); if (ld) { int d = atoi(ld); if (d > 0) L = enc_frame_len / d; } }
#endif
    const int OFFSET_MAX = 504;   /* (1<<9) - 8 */
    float xf = (float)(L - 50);
    float divisor;
    float offset_max = 30.0f;      /* reference default when no rate case matches */
    if (bit_depth == 24) {
        divisor = 4.0f;
        if (sample_rate == 96000)       offset_max = 25.7f - (25.7f - 20.75f) / 575.0f * xf;
        else if (sample_rate == 192000) offset_max = 25.7f - (25.7f - 24.42f) / 575.0f * xf;
        else if (sample_rate <= 48000)  offset_max = 26.2f - (26.2f  - 16.4f)  / 575.0f * xf; /* <=48k */
        /* else: 30.0 default */
    } else {
        divisor = 2.5f;
        if (sample_rate == 48000)       offset_max = 26.18f - (26.18f - 16.32f) / 575.0f * xf;
        else if (sample_rate <= 44100)  offset_max = 26.15f - (26.15f - 16.42f) / 575.0f * xf; /* <=44.1k */
        /* else (96k/192k 16-bit): 30.0 default */
    }
    int start = (int)(offset_max / divisor);          /* trunc toward zero */
#if defined(LHDC_HOST_BUILD)
    { const char *sp = getenv("LHDC_SPLIT"); if (sp) start = atoi(sp); }
#endif
    int deno = OFFSET_MAX - start - 1;                 /* = 503 - start */
    float jump = (deno != 0) ? (offset_max - (float)start) / (float)deno : 0.0f;
#if defined(LHDC_HOST_BUILD)
    { const char *sl = getenv("LHDC_SLOPE"); if (sl) jump = (float)atof(sl); }
#endif

    float e;
    if (gain_idx <= start) {
        e = (float)gain_idx;
    } else {
        int lo = (gain_idx < OFFSET_MAX) ? gain_idx : OFFSET_MAX;
        int hi = (gain_idx > OFFSET_MAX) ? gain_idx : OFFSET_MAX;
        e = (float)(lo - start) * jump + (float)start + (float)(hi - OFFSET_MAX);
        if (e > 30.0f) e = 30.0f;
    }
    return e;
}

/* Per-rate output level normalization. 44.1k/48k (mdct_size 480) decode at unity;
 * the reconstruction level falls ~1/N with the transform size N, so (mdct_size/480)
 * (480->x1, 960->x2, 1920->x4) restores a consistent, value-preserving level across
 * rates. NOTE: this is linear in N, not N^2. The previous (N/480)^2 over-amplified
 * the higher rates -- it was compensating the OLD (too-low) 96k/192k dequant gain;
 * with the exact per-rate gain table it drives 96k to full scale and 192k into
 * hard clipping. Linear rate_norm + the exact gain keeps every rate value-preserving. */
static float lhdc_dec_rate_norm(int mdct_size)
{
    float r = (float)mdct_size / 480.0f;
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_RATEPOW"); if (e) return powf(r, (float)atof(e)); }
#endif
    return r;
}

/* Per-config output level calibration toward value-preserving output (0 dBFS in
 * -> 0 dBFS out) so every LHDC config plays at the same volume regardless of
 * sample rate / bit depth. These trims cancel the residual decode gain that
 * remains after rate_norm, measured per config. They are approximate
 * (signal-dependent within ~1-2 dB); LHDC_LEVELDB overrides on the host build. */
static float lhdc_dec_level_cal(uint32_t sample_rate, int bit_depth)
{
    float db = 0.0f;
    if (bit_depth >= 24) {
        switch (sample_rate) {
            case 44100:  db = -1.32f; break;
            case 48000:  db = -0.21f; break;
            case 96000:  db = -1.29f; break;
            case 192000: db = +0.79f; break;
            default:     db = 0.0f;   break;
        }
    } else {
        db = -5.60f;   /* 16-bit (44.1/48k) */
    }
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_LEVELDB"); if (e) db = (float)atof(e); }
#endif
    return exp2f(db / 6.020599913f);   /* 10^(db/20) */
}

/*
 * Inverse quantization: linear dequant by the global gain step,
 * spectrum[k] = q[k] * 2^e(gain_idx). The SNS per-band shaping is undone
 * separately in lhdc_sns_synth_apply.
 */
static LHDC_HOT void lhdc_dec_inverse_quantize(int32_t *quant, float *spectrum,
                                       int num_coeffs,
                                       const lhdc_dec_sns_params_t *sns,
                                       int enc_frame_len, int bit_depth,
                                       uint32_t sample_rate)
{
    /* step = 2^e with FRACTIONAL e -> exp2f (single precision), NOT ldexpf
     * (which would truncate the fractional exponent and shift the level). */
    float step = exp2f(lhdc_dec_gain_exponent(
                           sns->global_gain, enc_frame_len, bit_depth, sample_rate));
    for (int k = 0; k < num_coeffs; k++) {
        spectrum[k] = (float)quant[k] * step;
    }
}

/*
 * Apply overlap-add for smooth frame transitions.
 * Uses 50% overlap with the previous frame.
 */
static LHDC_HOT void lhdc_dec_overlap_add(lhdc_decoder_t *dec, float *pcm_out,
                                  int channel, int mdct_size)
{
    int overlap = mdct_size / 2;
    const float *window = lhdc_dec_get_window(mdct_size);
    float *overlap_buf = dec->overlap_buf[channel];
    float *mdct_out = dec->mdct_out;

    /* Window the current IMDCT output */
    for (int n = 0; n < mdct_size; n++) {
        mdct_out[n] *= window[n];
    }

    /* Overlap-add: first half with previous frame overlap. Only `overlap`
     * (= mdct_size/2 = samples_per_channel) samples are emitted, so pcm_out is
     * exactly that long (= ch_pcm = w_buf, now half-size). */
    for (int n = 0; n < overlap; n++) {
        pcm_out[n] = mdct_out[n] + overlap_buf[n];
    }

    /* Save second half for next frame overlap. (The old code also wrote the
     * second half to pcm_out[overlap..mdct_size-1], but that range is never
     * emitted -- the caller reads only pcm_out[0..overlap-1] -- and is identical
     * to what we store in overlap_buf here, so it was dead writes that forced
     * w_buf to be full mdct_size. Dropped -> w_buf halves.) */
    for (int n = 0; n < overlap; n++) {
        overlap_buf[n] = mdct_out[overlap + n];
    }
}

/*
 * FAC moving-average window 1 (the stream-2 window is win1*8). win1 = payload/2
 * for high-bitrate frames (enc_frame_len >= 160), else payload/4. `enc_frame_len`
 * is the per-frame payload byte count (hdr->frame_bytes).
 */
static int lhdc_dec_ma_window(uint32_t sample_rate, int enc_frame_len)
{
    (void)sample_rate;
    int payload = enc_frame_len;
    int win1 = (enc_frame_len >= 160) ? (payload / 2) : (payload / 4);
    if (win1 < 1) win1 = 1;
    return win1;
}

/* Number of bits needed to represent n. Uses unsigned and a hard cap of 31 so a
 * large/garbage magnitude can never overflow. Computed directly via
 * __builtin_clz (count leading zeros): calc_bits(u) = (u==0) ? 0 :
 * floor(log2(u))+1, capped at 31. Hot path, called per coefficient in the
 * mantissa plane; `inline` folds it into the IRAM loop. */
static inline int lhdc_dec_calc_bits(int n)
{
    unsigned u = (n < 0) ? (unsigned)(-(long)n) : (unsigned)n;
    if (u == 0) return 0;
    int b = 32 - __builtin_clz(u);
    return (b > 31) ? 31 : b;
}

/*
 * Decode the mantissa+sign bit-plane for one channel into out[0..cnt-1] from the
 * post-Rice quotients coeff[0..cnt-1].
 *   M[k]  = (coeff[k] << shift[k]) | mantissa,  shift[k] from causal IIR predictor
 *   sign  = 1 extra bit when M != 0
 * `start_bit` is the plane start (relative to the channel byte slice). Returns the
 * peak |M| so the caller can select the correct plane offset: the range coder
 * leaves a data-dependent 2- or 3-byte lookahead gap, and a wrong start desyncs
 * the predictor and inflates |M|, so the smaller peak indicates the right offset.
 * pred_mode = flag2.
 */
static LHDC_HOT int64_t lhdc_dec_mant_plane(const int32_t *coeff, int32_t *out, int cnt,
                                   const uint8_t *pl, int total_bits,
                                   int start_bit, int pred_mode, int init_shift)
{
    int p = start_bit;
    /* The mantissa shift predictor is seeded by the transmitted initial shift
     * (the 4-bit "resid" field), not 0. shift[0] = (pred+0x40)>>7, and the first
     * iteration leaves pred unchanged, so seeding pred = init_shift<<7 yields
     * shift[0] = init_shift = calc_bits(|M[0]|). Without this, frames whose first
     * coefficient has a large magnitude and a zero Rice quotient lose all
     * mantissa bits, desyncing the spectrum. */
    int pred = init_shift << 7;
    int64_t peak = 0;
    for (int k = 0; k < cnt; k++) {
        int s = (pred + 0x40) >> 7;
        /* Read s mantissa bits MSB-first. When the 4 bytes are in bounds (the
         * common mid-buffer case) they are batched into a single shifted 4-byte
         * load: the field occupies bits (31-off)..(32-off-s), so (w<<off)>>(32-s)
         * right-aligns it. The guard (byte+4)*8<=total_bits ensures all 32 bits
         * are valid and s<=24 keeps off+s<=31. Boundary/large-s cases use the
         * per-bit fallback. */
        int mant;
        {
            int byte = p >> 3, off = p & 7;
            if (s == 0) {
                mant = 0;
            } else if (s <= 24 && ((byte + 4) << 3) <= total_bits) {
                uint32_t w = ((uint32_t)pl[byte] << 24) | ((uint32_t)pl[byte + 1] << 16) |
                             ((uint32_t)pl[byte + 2] << 8) | (uint32_t)pl[byte + 3];
                mant = (int)((w << off) >> (32 - s));
                p += s;
            } else {
                mant = 0;
                for (int b = 0; b < s; b++) {
                    int bit = (p < total_bits) ? ((pl[p >> 3] >> (7 - (p & 7))) & 1) : 0;
                    p++;
                    mant = (mant << 1) | bit;
                }
            }
        }
        int mag = ((int)coeff[k] << s) | mant;
        int val = mag;
        if (mag != 0) {
            int neg = (p < total_bits) ? ((pl[p >> 3] >> (7 - (p & 7))) & 1) : 0;
            p++;
            if (neg) val = -mag;
        }
        out[k] = val;
        { int64_t a = mag < 0 ? -(int64_t)mag : mag; if (a > peak) peak = a; }
        int x = lhdc_dec_calc_bits(mag) << 7;
#if defined(LHDC_HOST_BUILD)
        { extern volatile int g_lhdc_trace; if (g_lhdc_trace > 0 && k < 240)
            printf("[MANT] k=%d coeff=%d s=%d mant=%d mag=%d pred=%d p=%d\n", k, (int)coeff[k], s, mant, mag, pred, p); }
#endif
        pred = pred_mode ? ((7 * pred + x) >> 3) : ((5 * x + 3 * pred) >> 3);
    }
    return peak;
}

/*
 * Decode a single channel.
 *
 * force_sel: -1 = auto (descramble selector = header byte[8]&1, saving the
 * original header bytes for a possible retry); 0/1 = retry with this forced
 * selector (restoring the saved header bytes first). The byte[8]&1 selector is
 * wrong on a minority of frames, which desyncs that channel's FAC into a huge
 * |M|. The caller re-runs the channel with the flipped selector when the decode
 * goes out of range and keeps the in-range result. g_lhdc_chan_maxM exposes this
 * channel's peak |M| for that check.
 */
int64_t g_lhdc_chan_maxM = 0;
float g_lhdc_chan_specmax = 0;   /* peak of post-SNS spectrum; desync detector */
static lhdc_dec_ret_t lhdc_dec_decode_channel(lhdc_decoder_t *dec,
                                                int channel,
                                                float *pcm_out,
                                                int force_sel)
{
    lhdc_dec_bit_reader_t *br = &dec->bit_reader;
    lhdc_dec_frame_header_t *hdr = &dec->header;
    const lhdc_band_cfg_desc_t *band_cfg = dec->band_cfg;

    int mdct_size = band_cfg->mdct_size;
    int num_sfb = band_cfg->num_sfb;
    int num_coeffs = mdct_size / 2;
    int enc_frame_len = hdr->frame_bytes;
    int64_t t_chan0 = LHDC_NOW_US();
    int64_t t_ent0 = 0, t_ent1 = 0, t_im0 = 0, t_im1 = 0;

    /*
     * Each channel occupies a fixed, equal slice of the payload:
     * ch_bytes = frame_bytes / channels. The encoder writes two independent
     * byte-aligned channel buffers of frame_bytes/channels each. Channel
     * `channel` starts at ch_start = channel*ch_bytes. The bit reader is
     * positioned there and the channel's mantissa plane is confined to its own
     * slice (pl = payload+ch_start). */
    int n_ch = (hdr->channels < 1) ? 1 : hdr->channels;
    int ch0_bytes = enc_frame_len / n_ch;
    int ch_start = channel * ch0_bytes;
    int ch_bytes = (channel + 1 < n_ch) ? ch0_bytes : (enc_frame_len - ch_start);
#if defined(LHDC_HOST_BUILD)
    /* Override ch1's start byte to probe the real (possibly unequal) split. */
    if (channel == 1) { const char *e = getenv("LHDC_CH1_START");
        if (e) { ch_start = atoi(e); ch_bytes = enc_frame_len - ch_start; } }
#endif
    /* Each channel's 8-byte header is scrambled independently. parse_header only
     * copies the payload, so every channel descrambles its own header here, which
     * lets the caller retry the selector. */
    {
        static uint8_t s_hdr_save[8];
        static int s_saved_start = -1;
        uint8_t *cb = dec->payload_buf + ch_start;
        if (force_sel < 0) {
            if (ch_bytes >= 9) { for (int i = 0; i < 8; i++) s_hdr_save[i] = cb[i]; s_saved_start = ch_start; }
            lhdc_descramble_inplace(cb, (size_t)ch_bytes);              /* sel = cb[8]&1 */
        } else {
            if (s_saved_start == ch_start) { for (int i = 0; i < 8; i++) cb[i] = s_hdr_save[i]; }
            lhdc_descramble_inplace_sel(cb, (size_t)ch_bytes, force_sel);
        }
    }
    lhdc_bit_reader_init(br, dec->payload_buf + ch_start, (size_t)ch_bytes);

    /*
     * Per-channel leading section:
     *   [1] flag (==0)
     *   [9] global gain index
     *   [4] sns_mode-8
     *   [num_sfb-1] SNS direction bits   <- consumed inside lhdc_sns_decode_params
     *   [nzc_bits] (nzc>>1)-1            <- nzc_bits = calc_bits(samples)-1
     *   [1] flag2
     *   [4] residual field               <- present when frame_len >= 320
     * then the FAC spectral stream.
     */
    int flag = (int)lhdc_bit_reader_read(br, 1);
    if (flag != 0) {
        if (g_lhdc_trace > 0)
            ESP_LOGW("LHDCV5_DEC", "ch%d: flag!=0 (flag=%d) frame_bytes=%d mdct=%d sfb=%d",
                     channel, flag, enc_frame_len, mdct_size, num_sfb);
        return LHDC_DEC_BITSTREAM_ERROR;
    }
    dec->sns_params.global_gain = (int32_t)lhdc_bit_reader_read(br, 9);

    /* SNS: 4-bit sns_mode + (num_sfb-1) dir bits -> per-band scale factors. */
    lhdc_dec_ret_t ret = lhdc_sns_decode_params(br, &dec->sns_params, num_sfb);
    if (ret != LHDC_DEC_OK) {
        ESP_LOGW("LHDCV5_DEC", "ch%d: sns_decode ret=%d gain=%d sfb=%d",
                 channel, ret, (int)dec->sns_params.global_gain, num_sfb);
        return ret;
    }
    if (g_lhdc_trace > 0) {
        ESP_LOGI("LHDCV5_DEC", "ch%d leading ok: gain=%d sns_mode=%d num_sfb=%d frame_bytes=%d",
                 channel, (int)dec->sns_params.global_gain,
                 (int)dec->sns_params.sns_mode, num_sfb, enc_frame_len);
    }

    /* nzc field width = calc_bits(samples_per_frame) - 1
     * (7 for spf=240, 8 for 480, 9 for 960). */
    int nzc_bits = lhdc_dec_calc_bits(hdr->samples_per_channel) - 1;
    if (nzc_bits < 1) nzc_bits = 7;
    int nzc_raw = (int)lhdc_bit_reader_read(br, nzc_bits);
    int nzc = (nzc_raw + 1) * 2;
#if defined(LHDC_HOST_BUILD)
    { extern int g_nzc_formula; if (g_nzc_formula == 1) nzc = nzc_raw * 2; }
#endif
    if (g_lhdc_trace > 0 && channel == 0)
        printf("[I]LHDCV5_DEC: nzc_bits=%d nzc_raw=%d nzc=%d (clamp %d)\n",
               nzc_bits, nzc_raw, nzc, num_coeffs);
    if (nzc > num_coeffs) nzc = num_coeffs;
    dec->sns_params.nzc = nzc;

    /* flag2 selects the mantissa bit-allocation predictor: 0 = fast IIR
     * A[k]=(5*x+3*A[k-1])>>3, 1 = slow IIR B[k]=(7*B[k-1]+x)>>3. */
    int pred_mode = (int)lhdc_bit_reader_read(br, 1);   /* flag2 */
    /* The residual field is a 4-bit value read between flag2 and the FAC stream,
     * present only for larger frames. The threshold is enc_frame_len >= 320. */
    int resid_bits = (enc_frame_len >= 320) ? 4 : 0;
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_RESID"); if (e) resid_bits = atoi(e); }
#endif
    int resid_val = 0;
    if (resid_bits) resid_val = (int)lhdc_bit_reader_read(br, resid_bits);
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0)
        printf("[RESID] ch=%d resid_bits=%d resid_val=%d\n", channel, resid_bits, resid_val);
#endif
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_FACBIT");
      if (e) { int fb = atoi(e);
               lhdc_bit_reader_init(br, dec->payload_buf + ch_start, (size_t)ch_bytes);
               lhdc_bit_reader_skip(br, fb); } }
#endif

    /*
     * FAC entropy stream: snapshot the remaining payload as a byte buffer (the
     * range coder is byte-oriented and bit-packed from here, MSB-first), decode
     * the two ternary streams, and Rice-decode into |quant coeff|. Then the
     * sign plane (1 bit per nonzero coeff) follows at fac_start + fac_bytes.
     */
    int fac_start_bit = (int)lhdc_bit_reader_tell(br);
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0) {
        printf("[CHBYTES] ch=%d fac_start_bit=%d ch_start=%d ch_bytes=%d nzc=%d bytes:",
               channel, fac_start_bit, ch_start, ch_bytes, nzc);
        for (int b = 0; b < ch_bytes && b < 281; b++)
            printf(" %02x", dec->payload_buf[ch_start + b]);
        printf("\n");
        /* full payload + metadata for python replay of the real algorithm */
        char chpath[64]; snprintf(chpath, sizeof(chpath), "/data/local/tmp/lhdccal/chpay_ch%d.txt", channel);
        FILE *pf = fopen(chpath, "w");
        if (pf) {
            fprintf(pf, "fac_start_bit %d\nch_start %d\nch_bytes %d\nnzc %d\nflag2 %d\nresid_bits %d\n",
                    fac_start_bit, ch_start, ch_bytes, nzc, pred_mode, resid_bits);
            fprintf(pf, "payload:");
            for (int b = 0; b < ch_bytes; b++) fprintf(pf, " %02x", dec->payload_buf[ch_start + b]);
            fprintf(pf, "\n");
            fclose(pf);
        }
    }
#endif
    /*
     * Rice split = num_coeffs/3 (240->80, 480->160, 960->320), constant across
     * bitrate and bit depth. It sets pivot = max(count-split, 0); coefficients
     * beyond `pivot` switch to LSB-plane coding.
     */
    int split = num_coeffs / 3;
    /* FAC moving-average windows come from the band config (57/63 @48k,
     * 96/64 @96k); these are what the decoder's range model needs. */
    int ma_win1 = band_cfg->ma_win1;
    int ma_win2 = band_cfg->ma_win2;
    (void)lhdc_dec_ma_window;
#if defined(LHDC_HOST_BUILD)
    { const char *e = getenv("LHDC_MAWIN1"); if (e) { ma_win1 = atoi(e); ma_win2 = ma_win1 * 8; } }
    { const char *e = getenv("LHDC_MAWIN2"); if (e) ma_win2 = atoi(e); }
#endif
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0 && channel == 0) {
        FILE *pf = fopen("/data/local/tmp/lhdccal/chpay2.txt", "w");
        if (pf) { fprintf(pf, "split %d\nma_win1 %d\nma_win2 %d\n", split, ma_win1, ma_win2); fclose(pf); }
    }
#endif
    int fac_bytes = 0;
    t_ent0 = LHDC_NOW_US();
    ret = lhdc_entropy_decode_spectrum_ex2(dec->quant_spectrum,
                                          num_coeffs, br, nzc, split,
                                          ma_win1, ma_win2, &fac_bytes,
                                          dec->fac_buf, (int)sizeof(dec->fac_buf),
                                          dec->ent_s1,
                                          dec->ent_s2, dec->alloc_mdct_size);  /* ent_s2 capacity = num_coeffs*2 */
    t_ent1 = LHDC_NOW_US();
    if (ret != LHDC_DEC_OK) {
        return ret;
    }

    if (g_lhdc_trace > 0) {
        char buf[600]; int p = 0;
        p += snprintf(buf + p, sizeof(buf) - p, "ch%d COEFF(pre-sign)[0..47]:", channel);
        for (int k = 0; k < 48 && k < num_coeffs; k++)
            p += snprintf(buf + p, sizeof(buf) - p, " %d", (int)dec->quant_spectrum[k]);
        ESP_LOGI("LHDCV5_DEC", "%s", buf);
#if defined(LHDC_HOST_BUILD)
        /* dump full coeff list "idx val" to /tmp for reorder analysis */
        char cpath[64]; snprintf(cpath, sizeof(cpath), "/data/local/tmp/lhdccal/coeff_ch%d.txt", channel);
        FILE *cf = fopen(cpath, "w");
        if (cf) {
            for (int k = 0; k < num_coeffs; k++)
                fprintf(cf, "%d %d\n", k, (int)dec->quant_spectrum[k]);
            fclose(cf);
        }
#endif
    }

    /*
     * Mantissa + sign plane (immediately after the FAC byte stream at
     * fac_start + fac_bytes*8). The FAC stream carried only q[k] = the high bits
     * of each quantized coefficient. The full magnitude is reconstructed
     * sequentially:
     *
     *   shift[k] = calc_bits(|M[k-1]|)            (running context; M[-1]=0)
     *   M[k]     = (q[k] << shift[k]) | mantissa  (mantissa = shift[k] raw bits)
     *   if M[k] != 0: read 1 sign bit (1 = negative)
     *
     * The encoder writes shift[k] mantissa bits then a sign bit (when M[k]!=0),
     * MSB-first. Without this, the residual at high-energy bins (where q is
     * shifted to 0 and the magnitude lives entirely in the mantissa) is lost.
     */
    {
        /* The range coder leaves a data-dependent lookahead gap before the mantissa
         * plane: 2 or 3 bytes (16 or 24 bits), keyed on the final range, and it may
         * differ per channel. Decode the plane for both candidate starts and keep
         * the one with the smaller peak |M|: a wrong start desyncs the causal shift
         * predictor and inflates |M| by orders of magnitude. */
        const uint8_t *pl = dec->payload_buf + ch_start;   /* this channel's byte slice */
        int total_bits = ch_bytes * 8;
        int32_t *qs = dec->quant_spectrum;
        int cnt = nzc; if (cnt > num_coeffs) cnt = num_coeffs;
        int32_t *coeff = (int32_t *)dec->mdct_in;   /* scratch (free until inverse_quantize): post-Rice quotients */
        for (int k = 0; k < cnt; k++) coeff[k] = qs[k];

        int forced = -1, sel = 1;   /* sel: 0=min-peak, 1=range-rule (default) */
        /* All rates use the exact range-rule mantissa-gap selection. With the
         * correct FAC model windows the range coder stays in sync, so the rule
         * applies directly and the min-peak heuristic is not needed. */
#if defined(LHDC_HOST_BUILD)
        { const char *e = getenv("LHDC_MANT_DELTA"); if (e) forced = atoi(e); }
        { const char *e = getenv(channel == 0 ? "LHDC_D0" : "LHDC_D1"); if (e) forced = atoi(e); }
        { const char *e = getenv("LHDC_SEL"); if (e) sel = atoi(e); }
#endif
        extern uint32_t g_fac_final_range;
        int best_delta = 3;
        if (forced >= 0) {
            best_delta = forced;
        } else if (sel == 1) {
            /* exact arith_stop rule: leftover = 16 bits (2 bytes) if final range
             * <= 0x2000000, else 24 bits (3 bytes). */
            best_delta = (g_fac_final_range <= 0x2000000u) ? 2 : 3;
        } else {
            int64_t best_peak = -1;
            for (int d = 2; d <= 3; d++) {
                int fr = fac_bytes - d; if (fr < 0) fr = 0;
                int64_t peak = lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                                   fac_start_bit + fr * 8, pred_mode, resid_val);
                if (best_peak < 0 || peak < best_peak) { best_peak = peak; best_delta = d; }
            }
        }
#if defined(LHDC_HOST_BUILD)
        { if (getenv("LHDC_MANT_OFF")) { goto mant_done; } }
        { const char *e = getenv("LHDC_MANT_ABS");
          if (e) { int sb = atoi(e);
              (void)lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits, sb, pred_mode, resid_val);
              for (int k = cnt; k < num_coeffs; k++) qs[k] = 0;
              goto mant_done; } }
#endif
        {
            int fr = fac_bytes - best_delta; if (fr < 0) fr = 0;
            int64_t pk = lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                             fac_start_bit + fr * 8, pred_mode, resid_val);
            /* The 2-vs-3-byte gap can be mispicked on dense/high-bitrate frames;
             * a wrong start desyncs the shift predictor and pushes |M| past 24-bit
             * full scale. A correctly decoded frame cannot exceed full scale, so
             * if it does, retry the other gap and keep whichever stays in range.
             * Only the bad frames pay the extra pass. */
            if (forced < 0 && pk > 8388607) {
                int other = (best_delta == 2) ? 3 : 2;
                int fr2 = fac_bytes - other; if (fr2 < 0) fr2 = 0;
                int64_t pk2 = lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                                  fac_start_bit + fr2 * 8, pred_mode, resid_val);
                if (pk2 >= pk) {   /* other gap no better -> restore the range-rule result */
                    (void)lhdc_dec_mant_plane(coeff, qs, cnt, pl, total_bits,
                                              fac_start_bit + fr * 8, pred_mode, resid_val);
                } else {
                    best_delta = other;   /* qs already holds the better result */
                }
            }
        }
        for (int k = cnt; k < num_coeffs; k++) qs[k] = 0;
#if defined(LHDC_HOST_BUILD)
    mant_done:;
#endif
        /* Expose this channel's peak |M| so the caller can detect a desync (wrong
         * descramble selector) and retry with the flipped selector. */
        { int64_t mx = 0; for (int k = 0; k < cnt; k++) { int64_t a = qs[k] < 0 ? -qs[k] : qs[k]; if (a > mx) mx = a; }
          g_lhdc_chan_maxM = mx; }
        if (g_lhdc_trace) {
            ESP_LOGI("LHDCV5_DEC",
                     "ch%d: ch_start=%d ch_bytes=%d fac_start_bit=%d fac_bytes=%d best_delta=%d nzc=%d",
                     channel, ch_start, ch_bytes, fac_start_bit, fac_bytes, best_delta, nzc);
        }
        /* Next channel re-inits its own bit reader at its fixed slice (top of
         * decode_channel), so no boundary probing is needed here. */
    }

#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0) {
        char mpath[64]; snprintf(mpath, sizeof(mpath), "/data/local/tmp/lhdccal/Mdec_ch%d.txt", channel);
        FILE *mf = fopen(mpath, "w");
        if (mf) { for (int k = 0; k < num_coeffs; k++)
                      fprintf(mf, "%d %d\n", k, (int)dec->quant_spectrum[k]);
                  fclose(mf); }
    }
#endif

    /*
     * Coefficient de-reversal. The encoder quantizes the spectrum in reversed
     * bin order: its quant loop reads the whitened spectrum backward from bin
     * (nzc-1) down to 0 while writing the coeff array forward, so transmitted
     * coeff[j] = quantize(spectrum[nzc-1-j]). The decoder must reverse the first
     * nzc decoded coefficients back to natural bin order before dequant + SNS,
     * otherwise a low-frequency tone lands at a high bin. Bins [nzc..num_coeffs)
     * stay 0.
     */
    {
        int32_t *qs = dec->quant_spectrum;
        /* The decoded coefficients occupy qs[0..nzc-1], which is the encoder's
         * significant region in reversed order, so bin s comes from qs[nzc-1-s].
         * Reverse the first nzc entries in place; the dropped top spectrum lines
         * [nzc..num_coeffs) stay 0. */
        int rn = nzc; if (rn > num_coeffs) rn = num_coeffs;
        for (int a = 0, b = rn - 1; a < b; a++, b--) {
            int32_t t = qs[a]; qs[a] = qs[b]; qs[b] = t;
        }
    }

    /* Inverse quantization: linear dequant by 2^e(global_gain). The level table's
     * `size` parameter is the encoder's per-channel encoded byte count
     * (enc_pixel_size_byte), i.e. the frame payload divided by the channel count,
     * NOT the whole frame payload. */
    int bit_depth = (int)dec->config.bit_depth;
    int level_size = enc_frame_len / n_ch;
    lhdc_dec_inverse_quantize(dec->quant_spectrum,
                               dec->mdct_in, num_coeffs,
                               &dec->sns_params, level_size, bit_depth,
                               dec->config.sample_rate);
    if (g_lhdc_trace > 0) {
        int qmax = 0; float qsum = 0.0f; float dmax = 0;
        for (int k = 0; k < num_coeffs; k++) {
            int q = dec->quant_spectrum[k];
            if (q < 0) q = -q;
            if (q > qmax) qmax = q;
            qsum += q;
            float d = dec->mdct_in[k] < 0 ? -dec->mdct_in[k] : dec->mdct_in[k];
            if (d > dmax) dmax = d;
        }
        /* dominant bin (1kHz tone -> bin 10 @ 100Hz/bin); show its q and spectrum */
        int pk = 0; float pkv = 0;
        for (int k = 0; k < num_coeffs; k++) {
            float a = dec->mdct_in[k] < 0 ? -dec->mdct_in[k] : dec->mdct_in[k];
            if (a > pkv) { pkv = a; pk = k; }
        }
        ESP_LOGI("LHDCV5_DEC", "ch%d dequant: gain=%d e=%.2f qmean=%.3f specmax=%.1f peakbin=%d | q[8..12]=%d %d %d %d %d",
                 channel, (int)dec->sns_params.global_gain,
                 lhdc_dec_gain_exponent(dec->sns_params.global_gain, enc_frame_len, bit_depth, dec->config.sample_rate),
                 qsum / num_coeffs, dmax, pk,
                 (int)dec->quant_spectrum[8], (int)dec->quant_spectrum[9],
                 (int)dec->quant_spectrum[10], (int)dec->quant_spectrum[11],
                 (int)dec->quant_spectrum[12]);
    }

    if (g_lhdc_trace > 0 && channel == 0) {
        printf("[I]LHDCV5_DEC: PRESNS[0..%d]:", num_coeffs);
        for (int k = 0; k < num_coeffs; k++) printf(" %.0f", dec->mdct_in[k]);
        printf("\n");
    }

#if defined(LHDC_HOST_BUILD)
    if (channel == 0) {
        const char *sp = getenv("LHDC_DUMP_PRESPEC");
        if (sp) { FILE *spf = fopen(sp, "ab");
            if (spf) { fwrite(dec->mdct_in, sizeof(float), num_coeffs, spf); fclose(spf); } }
    }
#endif
    /* Apply SNS synthesis (inverse noise shaping) */
    lhdc_sns_synth_apply(dec->mdct_in, &dec->sns_params,
                          mdct_size / 2,
                          band_cfg->band_off, band_cfg->band_scale, num_sfb);
    if (g_lhdc_trace > 0) {
        float smax = 0; int pk = 0;
        for (int k = 0; k < num_coeffs; k++) {
            float d = dec->mdct_in[k] < 0 ? -dec->mdct_in[k] : dec->mdct_in[k];
            if (d > smax) { smax = d; pk = k; }
        }
        ESP_LOGI("LHDCV5_DEC", "ch%d after SNS: specmax=%.1f peakbin=%d sf[0..7]=%d %d %d %d %d %d %d %d",
                 channel, smax, pk, (int)dec->sns_params.scale_factors[0],
                 (int)dec->sns_params.scale_factors[1],
                 (int)dec->sns_params.scale_factors[2],
                 (int)dec->sns_params.scale_factors[3],
                 (int)dec->sns_params.scale_factors[4],
                 (int)dec->sns_params.scale_factors[5],
                 (int)dec->sns_params.scale_factors[6],
                 (int)dec->sns_params.scale_factors[7]);
    }

#if defined(LHDC_HOST_BUILD)
    /* Host diagnostic: append the IMDCT input (post-SNS spectrum) for ch0 to a
     * binary file for offline analysis. */
    if (channel == 0) {
        const char *sp = getenv("LHDC_DUMP_SPEC");
        if (sp) {
            FILE *spf = fopen(sp, "ab");
            if (spf) { fwrite(dec->mdct_in, sizeof(float), num_coeffs, spf); fclose(spf); }
        }
    }
#endif
#if defined(LHDC_HOST_BUILD)
    /* Print the peak magnitude at each decode stage (entropy / dequant / SNS) to
     * locate where an out-of-range frame originates. */
    if (getenv("LHDC_STAGEMAX")) {
        int qmax = 0; for (int k = 0; k < num_coeffs; k++) { int q = dec->quant_spectrum[k]; if (q<0) q=-q; if (q>qmax) qmax=q; }
        float smax = 0.0f; for (int k = 0; k < num_coeffs; k++) { float a = dec->mdct_in[k]; if (a<0) a=-a; if (a>smax) smax=a; }
        printf("[STAGEMAX] fr=%u ch=%d nzc_qmax=%d postSNS_specmax=%.0f gain=%d sns_mode=%d\n",
               dec->frame_index, channel, qmax, smax,
               (int)dec->sns_params.global_gain, (int)dec->sns_params.sns_mode);
    }
#endif
    /* IMDCT: reads w.mdct_in, writes u.mdct_out. After this w.mdct_in is dead. */
    t_im0 = LHDC_NOW_US();
    lhdc_imdct_transform(dec->mdct_in, dec->mdct_out, mdct_size);
    t_im1 = LHDC_NOW_US();
#if defined(LHDC_HOST_BUILD)
    if (channel == 0) { const char *mo = getenv("LHDC_DUMP_MDCTOUT");
        if (mo) { FILE *f = fopen(mo, "ab"); if (f) { fwrite(dec->mdct_out, sizeof(float), mdct_size, f); fclose(f); } } }
#endif

#if defined(LHDC_HOST_BUILD)
    float dbg_preov = 0.0f;
    if (getenv("LHDC_OVLDBG")) {
        const float *w = lhdc_dec_get_window(mdct_size);
        for (int n = 0; n < mdct_size/2; n++) { float a = dec->mdct_out[n]*w[n]; if (a<0) a=-a; if (a>dbg_preov) dbg_preov=a; }
    }
#endif
    /* Overlap-add: reads u.mdct_out + overlap_buf, writes pcm_out (= w.ch_pcm,
     * which aliases the now-dead w.mdct_in). overlap_add never reads mdct_in. */
    lhdc_dec_overlap_add(dec, pcm_out, channel, mdct_size);
#if defined(LHDC_HOST_BUILD)
    if (channel == 0) { const char *po = getenv("LHDC_DUMP_POSTOV");
        if (po) { FILE *f = fopen(po, "ab"); if (f) { fwrite(pcm_out, sizeof(float), dec->header.samples_per_channel, f); fclose(f); } } }
#endif
#if defined(LHDC_HOST_BUILD)
    if (getenv("LHDC_OVLDBG")) {
        float postov = 0.0f; int sp = dec->header.samples_per_channel;
        for (int n = 0; n < sp; n++) { float a = pcm_out[n]; if (a<0) a=-a; if (a>postov) postov=a; }
        printf("[OVLDBG] fr=%u ch=%d preov_peak=%.0f postov_peak=%.0f ratio=%.2f\n",
               dec->frame_index, channel, dbg_preov, postov, dbg_preov>0?postov/dbg_preov:0.0f);
    }
#endif

    /* Steady-state stage profile. Disabled by default: the periodic log output
     * blocks the decode task on the UART and disturbs the audio. Set
     * LHDCV5_DEC_PROFILE to 1 for bring-up only. */
#if LHDCV5_DEC_PROFILE
    if (g_lhdc_trace == 0) {
        static int64_t a_chan = 0, a_ent = 0, a_im = 0;
        static uint32_t a_n = 0;
        a_chan += LHDC_NOW_US() - t_chan0;
        a_ent  += t_ent1 - t_ent0;
        a_im   += t_im1 - t_im0;
        if (++a_n >= 400) {
            ESP_LOGI("LHDCV5_DEC",
                     "PROF (avg/chan over %u): total=%lld us  entropy=%lld us  imdct=%lld us  other=%lld us",
                     a_n, a_chan / a_n, a_ent / a_n, a_im / a_n,
                     (a_chan - a_ent - a_im) / a_n);
            a_chan = a_ent = a_im = 0; a_n = 0;
        }
    }
#else
    (void)t_chan0; (void)t_ent0; (void)t_ent1; (void)t_im0; (void)t_im1;
#endif

    return LHDC_DEC_OK;
}

/* Decode a channel, auto-correcting the descramble selector. The byte[8]&1
 * selector is correct for most frames but wrong for a minority, desyncing that
 * channel's FAC into a huge spectrum. Decode with the auto selector; if the
 * result is in range, keep it, otherwise re-decode with the flipped selector and
 * keep whichever is valid with the smaller peak. Only bad frames pay the extra
 * pass. decode_channel mutates overlap_buf via overlap-add, so the prior-frame
 * overlap is snapshotted and restored before each re-decode. */
/* Peak |sample| of a decoded channel (post overlap-add) in the internal scale. */
static float lhdc_chan_peak(const float *pcm, int n)
{
    float pk = 0.0f;
    for (int i = 0; i < n; i++) { float a = pcm[i] < 0 ? -pcm[i] : pcm[i]; if (a > pk) pk = a; }
    return pk;
}

static lhdc_dec_ret_t lhdc_dec_decode_channel_autosel(lhdc_decoder_t *dec,
                                                       int channel, float *pcm_out)
{
    /* Desync detector: the same condition the frame conceal uses, per channel:
     * |pcm|*sc > 2*clip_lim  ->  |pcm| > 2*clip_lim/sc, measured after overlap-add.
     * The overlap is restored before each retry so the decision uses the true
     * prior-frame context. */
    int bd = (int)dec->config.bit_depth;
    float sc = ldexpf(1.0f, bd - 23) * lhdc_dec_rate_norm(dec->band_cfg ? dec->band_cfg->mdct_size : 480)
             * lhdc_dec_level_cal(dec->header.sample_rate, bd);
    float clip_lim = (bd == 16) ? 32767.0f : (bd == 24) ? 8388607.0f : 2147483647.0f;
    const float PEAK_LIMIT = 2.0f * clip_lim / sc;
    int samples = dec->header.samples_per_channel;
    int n_ch = (dec->header.channels < 1) ? 1 : dec->header.channels;
    int ch0_bytes = dec->header.frame_bytes / n_ch;
    int ch_start = channel * ch0_bytes;
    int auto_sel = (ch_start + 8 < (int)sizeof(dec->payload_buf))
                 ? (dec->payload_buf[ch_start + 8] & 1) : 0;

    /* Snapshot the prior-frame overlap into the rate-sized ov_save buffer.
     * overlap_buf holds H = alloc_mdct_size/2 floats; cap to that. */
    int ov_n = samples;
    int ov_cap = dec->alloc_mdct_size / 2;
    if (ov_n > ov_cap) ov_n = ov_cap;
    float *s_ov_save = dec->ov_save;
    for (int i = 0; i < ov_n; i++) s_ov_save[i] = dec->overlap_buf[channel][i];

    lhdc_dec_ret_t r0 = lhdc_dec_decode_channel(dec, channel, pcm_out, -1);
    float p0 = (r0 == LHDC_DEC_OK) ? lhdc_chan_peak(pcm_out, samples) : 3.0e30f;
    if (r0 == LHDC_DEC_OK && p0 <= PEAK_LIMIT) return r0;   /* clean — common path */

    /* Suspect a wrong selector: try the flipped one from the saved overlap. */
    for (int i = 0; i < ov_n; i++) dec->overlap_buf[channel][i] = s_ov_save[i];
    lhdc_dec_ret_t r1 = lhdc_dec_decode_channel(dec, channel, pcm_out, auto_sel ^ 1);
    float p1 = (r1 == LHDC_DEC_OK) ? lhdc_chan_peak(pcm_out, samples) : 3.0e30f;
#if defined(LHDC_HOST_BUILD)
    if (getenv("LHDC_SELLOG"))
        printf("[SELLOG] ch%d auto_sel=%d r0=%d p0=%.0f r1=%d p1=%.0f limit=%.0f -> %s\n",
               channel, auto_sel, r0, p0, r1, p1, PEAK_LIMIT,
               (r1 == LHDC_DEC_OK && p1 < p0) ? "FLIP" : "keep-auto");
#endif

    if (r1 == LHDC_DEC_OK && p1 < p0) return r1;            /* flipped is better (pcm_out holds it) */

    /* Keep the auto result: restore overlap, re-decode with the auto selector. */
    for (int i = 0; i < ov_n; i++) dec->overlap_buf[channel][i] = s_ov_save[i];
    return lhdc_dec_decode_channel(dec, channel, pcm_out, auto_sel);
}

/* Public API */

/* Rate-sized tail layout (placed right after the struct): all 4-byte arrays
 * first (natural alignment), then the byte arrays. Mirrors lhdc_dec_carve(). */
static size_t lhdc_dec_tail_bytes(int mdct_size)
{
    int M = mdct_size, H = mdct_size / 2;
    size_t b = 0;
    b += (size_t)H * sizeof(float);                       /* w_buf (mdct_in[H spectrum]/ch_pcm[H pcm]) */
    b += (size_t)M * sizeof(float);                       /* u_buf (mdct_out[M]/quant[H]) */
    b += (size_t)LHDC_DEC_MAX_CHANNELS * (size_t)H * sizeof(float); /* overlap_buf */
    b += (size_t)H * sizeof(float);                       /* pcm_mid */
    b += (size_t)H * sizeof(float);                       /* ov_save (clip-recovery snapshot) */
    /* Entropy scratch sized to the actual bounds: ent_s1 is indexed only up to
     * num_coeffs (= mdct_size/2 = H), one per coefficient; ent_s2 (Rice pass-2
     * ternary) is filled only up to min(count*2, cap) <= num_coeffs*2 = M. */
    b += (size_t)H;                                       /* ent_s1 (num_coeffs) */
    b += (size_t)M;                                       /* ent_s2 (num_coeffs*2) */
    return b;
}

static int lhdc_dec_mdct_size(uint32_t sr, uint8_t dur)
{
    const lhdc_band_cfg_desc_t *bc = lhdc_get_band_cfg(sr, dur ? dur : 5);
    int M = bc ? (int)bc->mdct_size : 480;
    if (M <= 0 || M > LHDC_DEC_MAX_MDCT_SIZE) M = LHDC_DEC_MAX_MDCT_SIZE;
    return M;
}

/* Carve the tail buffers from `base` for the given mdct_size and point the
 * decoder's work pointers at them (preserving the mdct_in/ch_pcm and
 * mdct_out/quant_spectrum aliasing). Zeroes the overlap buffers. */
static void lhdc_dec_carve(lhdc_decoder_t *dec, uint8_t *base, int M)
{
    int H = M / 2;
    uint8_t *p = base;
    dec->mdct_in        = (float *)p;   p += (size_t)H * sizeof(float);
    dec->ch_pcm         = dec->mdct_in;                 /* alias (H: spectrum=mdct_size/2, pcm=samples/ch) */
    dec->mdct_out       = (float *)p;   p += (size_t)M * sizeof(float);
    dec->quant_spectrum = (int32_t *)dec->mdct_out;     /* alias */
    dec->overlap_buf[0] = (float *)p;   p += (size_t)H * sizeof(float);
    dec->overlap_buf[1] = (float *)p;   p += (size_t)H * sizeof(float);
    dec->pcm_mid        = (float *)p;   p += (size_t)H * sizeof(float);
    dec->ov_save        = (float *)p;   p += (size_t)H * sizeof(float);
    dec->ent_s1         = p;            p += (size_t)H;   /* num_coeffs */
    dec->ent_s2         = p;            p += (size_t)M;   /* num_coeffs*2 */
    dec->alloc_mdct_size = M;
    memset(dec->overlap_buf[0], 0, (size_t)LHDC_DEC_MAX_CHANNELS * (size_t)H * sizeof(float));
}

size_t lhdc_dec_get_workspace_size(uint32_t sample_rate, uint8_t frame_duration)
{
    return sizeof(lhdc_decoder_t) + lhdc_dec_tail_bytes(lhdc_dec_mdct_size(sample_rate, frame_duration));
}

lhdc_decoder_t *lhdc_dec_init(void *workspace, const lhdc_dec_config_t *config)
{
    lhdc_decoder_t *dec = (lhdc_decoder_t *)workspace;

    if (!dec) {
        return NULL;
    }

    memset(dec, 0, sizeof(*dec));

    if (config) {
        memcpy(&dec->config, config, sizeof(*config));
    } else {
        /* Default config - will be auto-detected from bitstream */
        dec->config.sample_rate    = LHDC_DEC_SR_48000;
        dec->config.bit_depth      = LHDC_DEC_BITDEPTH_24;
        dec->config.frame_duration = LHDC_DEC_FRAME_5MS;
        dec->config.channels       = 2;
        dec->config.max_frame_bytes = LHDC_DEC_MAX_FRAME_BYTES;
        dec->config.lossless_enable = 0;
    }

    /* Carve the rate-sized work buffers from the tail (immediately after the
     * struct). The caller sized the workspace with lhdc_dec_get_workspace_size()
     * for this same sample_rate/frame_duration. */
    {
        int M = lhdc_dec_mdct_size(dec->config.sample_rate, dec->config.frame_duration);
        lhdc_dec_carve(dec, (uint8_t *)workspace + sizeof(lhdc_decoder_t), M);
        /* Build this rate's IMDCT tables now and free the fast tables of any other
         * rate, so the larger tables do not linger on the heap after a downswitch
         * (e.g. 96k -> 48k). */
        lhdc_imdct_init(M);
    }

    /* Allocate the FAC entropy models at config time, while the heap still has a
     * large contiguous block. Allocating them lazily on the first decode can fail
     * at 96k, where Bluetooth streaming has already fragmented the heap. */
    lhdc_entropy_alloc();

    dec->initialized = 1;
    dec->header_parsed = 0;
    dec->frame_index = 0;

    return dec;
}

lhdc_dec_ret_t lhdc_dec_decode_frame(
    lhdc_decoder_t *dec,
    const uint8_t  *in_data,
    size_t          in_bytes,
    void           *out_pcm,
    uint32_t        out_samples,
    size_t         *consumed,
    uint32_t       *generated,
    lhdc_dec_frame_info_t *info)
{
    if (!dec || !dec->initialized) {
        return LHDC_DEC_NOT_INITIALIZED;
    }
    if (!in_data || in_bytes < 4 || !out_pcm || !consumed || !generated) {
        return LHDC_DEC_INVALID_PARAM;
    }

    *consumed = 0;
    *generated = 0;

    /*
     * Parse the [u16 LE][payload] frame header. On success the bit reader is
     * positioned at the start of the payload and frame_consumed = header +
     * payload bytes.
     */
    size_t frame_consumed = 0;
    lhdc_dec_ret_t ret = lhdc_dec_parse_header(dec, in_data, in_bytes,
                                               &frame_consumed);
    if (ret != LHDC_DEC_OK) {
        return ret;
    }

    dec->header_parsed = 1;

    /* Set up band configuration */
    dec->band_cfg = lhdc_get_band_cfg(dec->header.sample_rate, dec->header.frame_duration_ms);

    int mdct_size = dec->band_cfg->mdct_size;
    int samples_per_ch = dec->header.samples_per_channel;
    int channels = dec->header.channels;

    /* Check output buffer size */
    if ((uint32_t)samples_per_ch > out_samples) {
        return LHDC_DEC_BUF_NOT_ENOUGH;
    }

    /* Initialize IMDCT if needed */
    if (lhdc_imdct_init(mdct_size) != 0) {
        return LHDC_DEC_ERROR;
    }

    /* Window is produced on demand from the shared static cache (overlap-add). */
    (void)lhdc_dec_get_window(mdct_size);

    /* Output container width. The decode/gain math runs at the real source bit
     * depth (dec->header.bit_depth), but a 24-bit source is emitted in a 32-bit
     * int container (interleaved L/R) rather than 3-byte packed. This is
     * audio-neutral and gives a power-of-2 sample stride end-to-end, simplifying
     * the I2S/pipeline path. 16-bit is left as-is. */
    int out_bit_depth = ((int)dec->header.bit_depth == 24) ? 32 : (int)dec->header.bit_depth;
    int bytes_per_sample = out_bit_depth / 8;
    float *ch_pcm = dec->ch_pcm;   /* shared per-channel scratch (aliases mdct_in) */

    /*
     * Stereo. LHDC V5 transmits the two channels as direct L/R (ch0 = L,
     * ch1 = R), not mid/side. ch0's PCM is saved to pcm_mid before ch1 reuses
     * the shared ch_pcm scratch. Each channel's 8-byte header is descrambled
     * separately (see lhdc_descramble_inplace in decode_channel). Mono emits ch0.
     */
    ret = lhdc_dec_decode_channel_autosel(dec, 0, ch_pcm);
    if (ret != LHDC_DEC_OK) {
        return ret;
    }
    int do_stereo = (channels >= 2);
    if (do_stereo) {
        for (int n = 0; n < samples_per_ch; n++) dec->pcm_mid[n] = ch_pcm[n];
        ret = lhdc_dec_decode_channel_autosel(dec, 1, ch_pcm);   /* ch_pcm = right channel */
        if (ret != LHDC_DEC_OK) {
            /* Fall back to mono (mid to both) if ch1 fails. */
            for (int n = 0; n < samples_per_ch; n++) ch_pcm[n] = 0.0f;
        }
    }

    /* Output scale: the encoder works in a fixed 24-bit internal scale; combined
     * with the IMDCT/overlap reconstruction gain of 2 the total is
     * 2^(bit_depth-23). Brings output to the original level and keeps 16/24-bit
     * levels equal. */
    float sc = ldexpf(1.0f, out_bit_depth - 23) * lhdc_dec_rate_norm(mdct_size)
             * lhdc_dec_level_cal(dec->header.sample_rate, (int)dec->header.bit_depth);
    float clip_lim = (bytes_per_sample == 2) ? 32767.0f
                   : (bytes_per_sample == 3) ? 8388607.0f : 2147483647.0f;

    /*
     * Decode-garbage concealment. A correctly decoded frame can never exceed the
     * source's full-scale range, so a peak well past it (>2x) means this frame's
     * entropy/mantissa decode desynced and produced bogus coefficients (the
     * mantissa shift M=q<<s amplifies a desync into very large values). Such
     * frames are concealed with silence and the overlap is reset so the glitch
     * cannot smear into the next frame; a 5 ms mute is far less audible than the
     * resulting pop. */
    float frame_peak = 0.0f;
    int clip_cnt = 0;
    for (int n = 0; n < samples_per_ch; n++) {
        float l = do_stereo ? dec->pcm_mid[n] : ch_pcm[n];
        float r = ch_pcm[n];
        float al = (l < 0 ? -l : l) * sc, ar = (r < 0 ? -r : r) * sc;
        if (al > frame_peak) frame_peak = al;
        if (ar > frame_peak) frame_peak = ar;
        if (al > clip_lim) clip_cnt++;
        if (ar > clip_lim) clip_cnt++;
    }
    int conceal = (frame_peak > 2.0f * clip_lim);
#if defined(LHDC_HOST_BUILD)
    { extern int g_pkframe; float p0 = 0.0f, p1 = 0.0f;
        for (int n = 0; n < samples_per_ch; n++) {
            float a0 = (dec->pcm_mid[n] < 0 ? -dec->pcm_mid[n] : dec->pcm_mid[n]) * sc;
            float a1 = (ch_pcm[n] < 0 ? -ch_pcm[n] : ch_pcm[n]) * sc;
            if (a0 > p0) p0 = a0; if (a1 > p1) p1 = a1;
        }
        if (getenv("LHDC_PKALL") || conceal)
            printf("[PK] frame=%d ch0peak=%.0f ch1peak=%.0f %s\n",
                   g_pkframe, p0, p1, conceal ? (p1>2*clip_lim?"CONCEAL-ch1":"CONCEAL-ch0") : "");
        g_pkframe++;
    }
#endif
    if (conceal) {
        /* Zero the full overlap buffers (H = alloc_mdct_size/2 floats each) so no
         * stale overlap leaks into the next frame after a concealed error. */
        size_t ov_bytes = (size_t)(dec->alloc_mdct_size / 2) * sizeof(float);
        memset(dec->overlap_buf[0], 0, ov_bytes);
        if (channels >= 2) memset(dec->overlap_buf[1], 0, ov_bytes);
    }

    /* Interleave to output (l/r already include the bit-depth scale `sc`). */
    #define LHDC_OUT_SAMPLE(dst, fval, lo, hi) do { \
        float _s = (fval); if (_s > (hi)) _s = (hi); if (_s < (lo)) _s = (lo); \
        (dst) = (int32_t)_s; } while (0)
    for (int n = 0; n < samples_per_ch; n++) {
        float l, r;
        if (conceal)        { l = r = 0.0f; }
        else if (do_stereo) { l = dec->pcm_mid[n] * sc; r = ch_pcm[n] * sc; }
        else                { l = r = ch_pcm[n] * sc; }
        int32_t lv, rv;
        if (bytes_per_sample == 2) {
            LHDC_OUT_SAMPLE(lv, l, -32768.0f, 32767.0f);
            LHDC_OUT_SAMPLE(rv, r, -32768.0f, 32767.0f);
            int16_t *o = (int16_t *)out_pcm;
            o[n * channels + 0] = (int16_t)lv;
            if (channels >= 2) o[n * channels + 1] = (int16_t)rv;
        } else if (bytes_per_sample == 3) {
            /*
             * 24-bit PCM output for ESP32 I2S/DMA, written as interleaved 32-bit
             * slots (int32 L, int32 R, ...) rather than 3-byte packed samples,
             * which matches the common ESP32 I2S configuration that carries
             * 24-bit audio in 32-bit slots. The 24 valid bits are left-justified
             * by default (sample << 8); define LHDC_DEC_24BIT_I2S_RIGHT_JUSTIFIED
             * to keep them right-justified in the low 24 bits instead.
             *
             * Caller note: output bytes = samples_per_ch * channels *
             * sizeof(int32_t), not * 3.
             */
            LHDC_OUT_SAMPLE(lv, l, -8388608.0f, 8388607.0f);
            LHDC_OUT_SAMPLE(rv, r, -8388608.0f, 8388607.0f);
            int32_t *o = (int32_t *)out_pcm;
#if defined(LHDC_DEC_24BIT_I2S_RIGHT_JUSTIFIED) && LHDC_DEC_24BIT_I2S_RIGHT_JUSTIFIED
            o[n * channels + 0] = lv;
            if (channels >= 2) o[n * channels + 1] = rv;
#else
            o[n * channels + 0] = (int32_t)(lv << 8);
            if (channels >= 2) o[n * channels + 1] = (int32_t)(rv << 8);
#endif
        } else if (bytes_per_sample == 4) {
            LHDC_OUT_SAMPLE(lv, l, -2147483648.0f, 2147483647.0f);
            LHDC_OUT_SAMPLE(rv, r, -2147483648.0f, 2147483647.0f);
            int32_t *o = (int32_t *)out_pcm;
            o[n * channels + 0] = lv;
            if (channels >= 2) o[n * channels + 1] = rv;
        }
    }
    #undef LHDC_OUT_SAMPLE

    /* Stats: concealed (garbage) frames and clipping, rate-limited. */
    {
        static uint32_t s_conceal = 0, s_clip_frames = 0, s_total = 0, s_rl = 0;
        s_total++;
        if (conceal)      s_conceal++;
        if (clip_cnt > 0) s_clip_frames++;
        if (conceal && (s_rl++ % 16) == 0) {
            ESP_LOGW("LHDCV5_DEC",
                     "CONCEAL frame#%u: peak=%.0f (lim=%.0f) | concealed=%u/%u clipframes=%u",
                     (unsigned)dec->frame_index, frame_peak, clip_lim,
                     s_conceal, s_total, s_clip_frames);
        }
    }

    /* Report bytes consumed (u16 LE header + payload) and samples generated. */
    *consumed = frame_consumed;
    *generated = (uint32_t)samples_per_ch;
    dec->frame_index++;

    if (g_lhdc_trace > 0) {
        g_lhdc_trace--;
        ESP_LOGI("LHDCV5_DEC", "FRAME ok: consumed=%u frame_bytes=%d gen=%u ch=%d",
                 (unsigned)frame_consumed, (int)dec->header.frame_bytes,
                 (unsigned)samples_per_ch, channels);
    }

    /* Fill frame info if requested */
    if (info) {
        info->frame_index        = dec->frame_index - 1;
        info->encoded_frame_bytes = dec->header.frame_bytes;
        info->samples_per_channel = dec->header.samples_per_channel;
        info->channels           = dec->header.channels;
        info->sample_rate        = dec->header.sample_rate;
        info->bit_depth          = (uint8_t)out_bit_depth;   /* emitted container width (24->32) */
        info->frame_duration_ms  = dec->header.frame_duration_ms;
        info->version            = dec->header.version;
        info->ext_func_flags     = dec->header.ext_func_flags;
        info->target_bitrate     = dec->header.target_bitrate;
    }

    return LHDC_DEC_OK;
}

void lhdc_dec_flush(lhdc_decoder_t *dec)
{
    if (!dec) return;

    /* Clear the full overlap buffers (H = alloc_mdct_size/2 floats each) so stale
     * overlap from a previous stream cannot leak into the first frame after a
     * flush/reset (codec switch / track start) and cause a cold-start click. */
    size_t ov_bytes = (size_t)(dec->alloc_mdct_size / 2) * sizeof(float);
    if (ov_bytes == 0) ov_bytes = (size_t)(LHDC_DEC_MAX_MDCT_SIZE / 2) * sizeof(float);
    for (int ch = 0; ch < LHDC_DEC_MAX_CHANNELS; ch++) {
        if (dec->overlap_buf[ch]) memset(dec->overlap_buf[ch], 0, ov_bytes);
    }
}

void lhdc_dec_reset(lhdc_decoder_t *dec)
{
    if (!dec) return;

    lhdc_dec_flush(dec);
    dec->header_parsed = 0;
    dec->frame_index = 0;
    memset(&dec->header, 0, sizeof(dec->header));
    memset(&dec->sns_params, 0, sizeof(dec->sns_params));
}

lhdc_dec_ret_t lhdc_dec_get_config(lhdc_decoder_t *dec, lhdc_dec_config_t *config)
{
    if (!dec || !config) {
        return LHDC_DEC_INVALID_PARAM;
    }

    if (dec->header_parsed) {
        config->sample_rate    = (lhdc_dec_sample_rate_t)dec->header.sample_rate;
        config->bit_depth      = (lhdc_dec_bitdepth_t)dec->header.bit_depth;
        config->frame_duration = (lhdc_dec_frame_duration_t)dec->header.frame_duration_ms;
        config->channels       = dec->header.channels;
        config->max_frame_bytes = dec->header.frame_bytes;
        config->lossless_enable = dec->header.lossless;
    } else {
        memcpy(config, &dec->config, sizeof(*config));
    }

    return LHDC_DEC_OK;
}

const char *lhdc_dec_strerror(lhdc_dec_ret_t ret)
{
    switch (ret) {
        case LHDC_DEC_OK:                  return "Success";
        case LHDC_DEC_ERROR:               return "Generic error";
        case LHDC_DEC_INVALID_PARAM:       return "Invalid parameter";
        case LHDC_DEC_INVALID_HANDLE:      return "Invalid handle";
        case LHDC_DEC_NOT_INITIALIZED:     return "Decoder not initialized";
        case LHDC_DEC_BUF_NOT_ENOUGH:      return "Output buffer not large enough";
        case LHDC_DEC_BITSTREAM_ERROR:     return "Bitstream error";
        case LHDC_DEC_UNSUPPORTED_VERSION: return "Unsupported LHDC version";
        case LHDC_DEC_UNSUPPORTED_SR:      return "Unsupported sample rate";
        case LHDC_DEC_UNSUPPORTED_FORMAT:  return "Unsupported format";
        case LHDC_DEC_NEED_MORE_DATA:      return "Need more input data";
        default:                           return "Unknown error";
    }
}
