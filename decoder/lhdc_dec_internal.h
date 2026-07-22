#ifndef LHDC_DEC_INTERNAL_H
#define LHDC_DEC_INTERNAL_H

#include "lhdc_dec.h"
#include <stdint.h>

/* Forward declaration - defined in lhdc_tables.h */
typedef struct lhdc_band_cfg_desc lhdc_band_cfg_desc_t;

/*
 * MAX_MDCT_SIZE = max samples-per-frame the decoder supports, and the single
 * biggest driver of the heap workspace. Per lhdc_get_band_cfg:
 * 480 -> 44.1/48k@5ms, 960 -> 96k, 1920 -> 192k.
 * Capped at 480 (44.1/48 kHz, 24-bit) — the rate phones actually negotiate — to
 * minimise RAM. This halves every per-sample work array vs. 96 kHz, getting the
 * whole workspace to ~10 KB so playback free heap stays near LDAC levels.
 */
#define LHDC_DEC_MAX_MDCT_SIZE       1920   /* 96k (mdct 960); 48k uses rate-sized 480 */
#define LHDC_DEC_MAX_BANDS           32    /* HR uses 32 bands */
#define LHDC_DEC_MAX_SFB             32
#define LHDC_DEC_MAX_FRAME_BYTES     1536   /* 48k/5ms HR payload <=~660B */
#define LHDC_DEC_MAX_CHANNELS        2
#define LHDC_DEC_OVERLAP_SIZE        (LHDC_DEC_MAX_MDCT_SIZE / 2)
#define LHDC_DEC_HEADER_MAX_BYTES    64
#define LHDC_DEC_SYNC_WORD           0x4C48

typedef struct {
    const uint8_t *data;
    size_t         data_bytes;
    size_t         byte_pos;
    uint64_t       cache;
    int            cache_bits;
} lhdc_dec_bit_reader_t;

typedef struct {
    uint16_t  sync_word;
    uint8_t   version;
    uint8_t   frame_duration_ms;
    uint32_t  sample_rate;
    uint8_t   channels;
    uint8_t   bit_depth;
    uint32_t  ext_func_flags;
    uint32_t  target_bitrate;
    uint16_t  frame_bytes;
    uint16_t  frame_flags;     /* hdr>>10 from the [u16 LE] frame header (§5) */
    uint16_t  samples_per_channel;
    uint8_t   band_cfg_idx;
    uint8_t   quality_level;
    uint8_t   lossless;
    uint16_t  meta_len;
    uint8_t   meta_data[LHDC_DEC_HEADER_MAX_BYTES];
} lhdc_dec_frame_header_t;

typedef struct {
    int32_t  scale_factors[LHDC_DEC_MAX_SFB];
    uint8_t  num_sfb;
    int32_t  sns_mode;      /* 4-bit field (8..23), transmitted as sns_mode-8 */
    int32_t  global_gain;   /* 9-bit global quantizer step index (read by caller) */
    int32_t  nzc;           /* significant-coefficient count (read by caller) */
    int32_t  noise_level[LHDC_DEC_MAX_SFB];
} lhdc_dec_sns_params_t;

struct lhdc_decoder_t {
    lhdc_dec_config_t       config;
    lhdc_dec_frame_header_t header;
    lhdc_dec_sns_params_t   sns_params;
    const lhdc_band_cfg_desc_t *band_cfg;

    /*
     * Work buffers, aggressively shared to minimise RAM. Decoding is sequential
     * per channel, so single-channel scratch suffices. The pipeline per channel:
     *   entropy -> u.quant_spectrum   (int32 |coeff|)
     *   inverse_quantize: u.quant_spectrum -> mdct_in   (float spectrum)
     *   sns_synth: mdct_in in place
     *   imdct: mdct_in -> u.mdct_out                    (u reused as float out)
     *   overlap_add: u.mdct_out + overlap_buf -> ch_pcm (mdct_in now dead)
     *
     * Two unions exploit non-overlapping lifetimes:
     *  - u: quant_spectrum (entropy out, dead after inverse_quantize) shares with
     *    mdct_out (IMDCT out, written later).
     *  - mdct_in shares with ch_pcm: mdct_in is dead once the IMDCT has read it,
     *    which is exactly when overlap_add starts writing ch_pcm.
     * Saves 2 * MAX_MDCT floats vs. naive layout.
     */
    /*
     * RATE-SIZED work buffers: these are pointers into a single tail block
     * allocated right after this struct, sized to the negotiated mdct_size (see
     * lhdc_dec_get_workspace_size / lhdc_dec_init). So 48k (mdct 480) uses ~8 KB
     * of buffers while 96k (mdct 960) uses ~16 KB, instead of every rate paying
     * the worst case. The union aliasing is preserved by pointing the alias
     * members at the same sub-buffer:
     *   mdct_in / ch_pcm     -> shared w_buf  (lifetimes don't overlap)
     *   mdct_out / quant_spectrum -> shared u_buf
     */
    float   *mdct_in;        /* = w_buf : spectrum + IMDCT input */
    float   *ch_pcm;         /* = w_buf : per-channel PCM output (aliases mdct_in) */
    float   *mdct_out;       /* = u_buf : IMDCT time output */
    int32_t *quant_spectrum; /* = u_buf : entropy |coeff| (aliases mdct_out) */
    float   *overlap_buf[LHDC_DEC_MAX_CHANNELS];
    float   *pcm_mid;        /* ch0 (mid) PCM held while ch1 decodes; stereo only */
    float   *ov_save;        /* clip-recovery overlap snapshot (H floats); was .bss */
    int      alloc_mdct_size;/* mdct_size the tail buffers were carved for */

    lhdc_dec_bit_reader_t bit_reader;
    uint8_t  payload_buf[LHDC_DEC_MAX_FRAME_BYTES];  /* descrambled frame payload */

    /* Entropy-decode scratch. fac_buf holds the FAC byte stream (one channel's
     * bytes, well under 1.5KB). ent_s1 = pass-1 ternary (one per coeff). ent_s2 =
     * pass-2 quotient ternary; both rate-sized in the tail block. */
    uint8_t  fac_buf[768];   /* one channel's FAC bytes <= frame_bytes/2 <= MAX_FRAME_BYTES/2 = 768 */
    uint8_t  *ent_s1;
    uint8_t  *ent_s2;

    uint32_t frame_index;
    uint8_t  initialized;
    uint8_t  header_parsed;
};

#endif
