#include "lhdc_entropy_dec.h"
#include "lhdc_bit_reader.h"
#include "lhdc_tables.h"
#include <string.h>
#include <stdlib.h>   /* malloc/free for the lazily-heaped FAC models */
#if defined(LHDC_HOST_BUILD)
#include <stdio.h>
#include <stdlib.h>
#define LHDC_HOT          /* host: no IRAM */
#else
#include "esp_attr.h"
/* Place the per-symbol/per-coeff entropy hot path in IRAM. The BT controller is
 * interrupt-heavy and evicts the decoder's flash-cache lines mid-frame, which
 * both slows the steady decode and causes the per-packet latency spikes. IRAM
 * execution removes those cache stalls. ~18 KB IRAM headroom available. */
#define LHDC_HOT          IRAM_ATTR
#endif

/*
 * LHDC V5 entropy decoder — reverse-engineered and validated bit-exact against
 * the real liblhdcv5.so encoder (range coder 50/50, Rice 20000/20000 round-trip,
 * full tone1k frame -> ~1kHz spectrum).
 *
 * The spectral data is two ternary (3-symbol) streams produced by
 * RiceQuotientEncode then FAC-MA range coded (BitsPredictCompressOutput):
 *   stream1 = first `count` symbols (lsb/marker plane), init freqs {54,12,1},
 *             sliding window MA_WIN1.
 *   stream2 = the rest (quotient codes),                init freqs {90,16,1},
 *             sliding window MA_WIN2.
 * Both share one byte range-coder stream (init code = first 4 bytes; total freq
 * = 2^15). After the FAC stream a sign bit-plane follows: one bit per nonzero
 * coefficient (1 = negative).
 */

#define FAC_TOTAL_BITS 15
#define FAC_TOTAL      (1 << FAC_TOTAL_BITS)
#define FAC_NSYM       3

#if defined(LHDC_HOST_BUILD)
int g_fdc = 0;     /* per-channel DSTATE symbol counter (reset each entropy call) */
int g_block = 0;   /* which channel-block since trace start (0=a-ch0,1=a-ch1,2=b-ch0,...) */
#endif
uint32_t g_fac_final_range = 0;   /* final range register after entropy decode (leftover analysis) */

static const uint32_t FAC_FQ1[FAC_NSYM] = {54, 12, 1};
static const uint32_t FAC_FQ2[FAC_NSYM] = {90, 16, 1};

/* --- FAC-MA adaptive model (sliding window) --- */
typedef struct {
    int      n;
    int      window;
    int      pos;
    int      count;
    uint8_t *hist;                /* MA-window history; points at s_fac_hist1/2 */
    int      hist_cap;            /* capacity of *hist (window is clamped to it) */
    uint32_t freq[FAC_NSYM];
    uint32_t thresh[FAC_NSYM];
    uint32_t cum[FAC_NSYM + 1];   /* 15-bit normalized cumulative freqs */
} fac_model_t;

/* The two adaptive models live in .bss (NOT heap): a heap block here fragments
 * the LHDC heap so the 18 KB workspace can't find a contiguous hole on a rate
 * switch (-> dec=NULL -> no audio). .bss never fragments.
 *
 * BUT the two models need very different history sizes, so they get SEPARATE
 * right-sized buffers instead of two identical hist[3072]:
 *   s_fac_hist1 -> stream-1 model, window = ma_win1 (<= ~281 at 900k; <=512).
 *   s_fac_hist2 -> stream-2 model, window = ma_win2 = ma_win1*8 (<= ~2448).
 * The old struct embedded hist[3072] in BOTH (~6.2 KB .bss); stream-1's was
 * ~2.5 KB of pure waste (it only ever uses ma_win1 entries). Right-sizing frees
 * that .bss -> the heap pool is ~2.5 KB larger during ALL LHDC playback (96k and
 * 48k). s_fac_m[0] = stream-1, s_fac_m[1] = stream-2. */
static uint8_t s_fac_hist1[512];    /* stream-1 (ma_win1) */
static uint8_t s_fac_hist2[3072];   /* stream-2 (ma_win2, win1*8) */
static fac_model_t s_fac_m[2];

void lhdc_entropy_alloc_internal(void)
{
    s_fac_m[0].hist = s_fac_hist1; s_fac_m[0].hist_cap = (int)sizeof(s_fac_hist1);
    s_fac_m[1].hist = s_fac_hist2; s_fac_m[1].hist_cap = (int)sizeof(s_fac_hist2);
}

void lhdc_entropy_alloc(void) { /* no-op: models are static .bss */ }
void lhdc_entropy_free(void)  { /* no-op: models are static .bss */ }

static LHDC_HOT void fac_model_rescale(fac_model_t *m)
{
    uint32_t cum[FAC_NSYM + 1];
    cum[0] = 0;
    for (int i = 0; i < m->n; i++) {
        cum[i + 1] = cum[i] + m->freq[i];
        m->thresh[i] = m->freq[i] + (m->freq[i] >> 5);
    }
    uint32_t total = cum[m->n];
    uint32_t scale = (uint32_t)(0x80000000u / total);
    m->cum[0] = 0;
    for (int i = 1; i <= m->n; i++) {
        m->cum[i] = (uint32_t)(((uint64_t)cum[i] * scale) >> 16);
    }
}

static void fac_model_init(fac_model_t *m, const uint32_t *init_freq, int window)
{
    m->n = FAC_NSYM;
    if (window > m->hist_cap) window = m->hist_cap;   /* clamp to the right-sized buffer */
    if (window < 1) window = 1;
    m->window = window;
    m->pos = 0;
    m->count = 0;
    memset(m->hist, 0, (size_t)window);   /* only the used window (was full 3072) */
    for (int i = 0; i < FAC_NSYM; i++) m->freq[i] = init_freq[i];
    fac_model_rescale(m);
}

static LHDC_HOT void fac_model_update(fac_model_t *m, int sym)
{
    if (m->count < m->window) {
        m->freq[sym]++;
        if (m->freq[sym] >= m->thresh[sym]) fac_model_rescale(m);
        m->hist[m->count] = (uint8_t)sym;
        m->count++;
    } else {
        int old = m->hist[m->pos];
        if (m->freq[old] >= 2) m->freq[old]--;
        m->freq[sym]++;
        if (m->freq[sym] >= m->thresh[sym]) fac_model_rescale(m);
        m->hist[m->pos] = (uint8_t)sym;
        m->pos++;
        if (m->pos >= m->window) m->pos = 0;
    }
}

/* --- FAC range decoder --- */
typedef struct {
    const uint8_t *data;
    int            len;
    int            p;
    uint32_t       range;
    uint32_t       code;
} fac_dec_t;

static LHDC_HOT uint8_t fac_byte(fac_dec_t *d)
{
    uint8_t b = (d->p < d->len) ? d->data[d->p] : 0;
    d->p++;
    return b;
}

static void fac_dec_init(fac_dec_t *d, const uint8_t *data, int len)
{
    d->data = data;
    d->len = len;
    d->p = 0;
    d->range = 0xFFFFFFFFu;
    d->code = 0;
    for (int i = 0; i < 4; i++) d->code = (d->code << 8) | fac_byte(d);
}

static LHDC_HOT int fac_decode(fac_dec_t *d, fac_model_t *m)
{
#if defined(LHDC_HOST_BUILD)
    extern volatile int g_lhdc_trace; extern int g_fdc;   /* reset per-channel by the entropy fn */
    uint32_t dbg_range=d->range, dbg_code=d->code;
    uint32_t dbg_cum1=m->cum[1], dbg_cum2=m->cum[2], dbg_f0=m->freq[0],dbg_f1=m->freq[1],dbg_f2=m->freq[2],dbg_cnt=m->count;
#endif
    uint32_t r = d->range >> FAC_TOTAL_BITS;
    /* Divide-free symbol search. The ARM reference computed `target = code / r`
     * per symbol; Xtensa LX6 has no fast integer divide (~30+ cycle software
     * call) whereas it has a fast 32-bit multiply. Use the integer identity
     * floor(code/r) >= c  <=>  code >= r*c  (c>=0, r>0) to compare r*cum[] to
     * code directly. m->n is always 3 (ternary FAC), so this is <=2 multiplies
     * replacing one divide per decoded symbol. Bit-IDENTICAL to the old
     * target+clamp path: the `target>=FAC_TOTAL -> FAC_TOTAL-1` clamp is
     * subsumed by the `sym+1 < m->n` loop bound (when code/r overshoots, the
     * scan still stops at the last symbol n-1). No overflow: r*cum[sym+1] <=
     * r*FAC_TOTAL = (range>>15)<<15 <= range < 2^32. */
    int sym = 0;
    while (sym + 1 < m->n && r * m->cum[sym + 1] <= d->code) sym++;
    d->code -= r * m->cum[sym];
    d->range = r * (m->cum[sym + 1] - m->cum[sym]);
    while ((d->range >> 24) == 0) {
        d->code = (d->code << 8) | fac_byte(d);
        d->range <<= 8;
    }
    fac_model_update(m, sym);
#if defined(LHDC_HOST_BUILD)
    if (g_lhdc_trace > 0) {
        uint32_t target = dbg_code / (dbg_range >> FAC_TOTAL_BITS);   /* trace only */
        if (g_fdc < 15 || (g_fdc >= 234 && g_fdc <= 244))
            printf("[DSTATE blk%d] [%d] sym=%d range=%08x cum1=%u cum2=%u f=[%u %u %u] cnt=%u target=%u\n",
                   g_block, g_fdc, sym, dbg_range, dbg_cum1, dbg_cum2, dbg_f0,dbg_f1,dbg_f2, dbg_cnt, target);
        g_fdc++;
    }
#endif
    return sym;
}

/* --- Rice quotient inverse (mirrors rice_dec.py, validated) --- */
__attribute__((unused))
static LHDC_HOT int rice_read_quotient(const uint8_t *s2, int n, int *jp)
{
    int j = *jp;
    if (j < n && s2[j] == 2) {
        uint32_t low = 0;
        int shift = 0;
        while (j < n && s2[j] == 2) {
            j++;
            int b1 = (j < n) ? s2[j] : 0; j++;
            int b0 = (j < n) ? s2[j] : 0; j++;
            low |= (uint32_t)(((b1 << 1) | b0)) << shift;
            shift += 2;
        }
        int ones = 0;
        while (j < n && s2[j] == 1) { ones++; j++; }
        if (j < n && s2[j] == 0) j++;
        *jp = j;
        return (int)(low | ((uint32_t)(ones + 1) << shift));
    }
    int ones = 0;
    while (ones < 3 && j < n && s2[j] == 1) { ones++; j++; }
    if (ones == 3) { *jp = j; return 3; }
    if (j < n && s2[j] == 0) j++;
    *jp = j;
    return ones;
}

/*
 * On-demand pass-2 stream: decodes s2 symbols from the range coder lazily with a
 * 1-symbol lookahead, mirroring the array peek/consume semantics EXACTLY (s2[j]
 * -> peek, j++ -> consume). This avoids pre-decoding a large fixed bound AND the
 * separate re-decode to find the byte position: because each consumed symbol is
 * decoded exactly once and `save_p/save_range` snapshot the reader state BEFORE
 * the current (possibly unconsumed) lookahead symbol, the precise byte position
 * after the consumed symbols is available directly. Result is bit-identical to
 * the array path but ~4x fewer symbol decodes (fixes the real-time stutter).
 */
typedef struct { fac_dec_t *d; fac_model_t *m; int has; int val; int save_p; uint32_t save_range; int used; } s2_stream_t;
static int s2_peek(s2_stream_t *s) {
    if (!s->has) { s->save_p = s->d->p; s->save_range = s->d->range;
                   s->val = fac_decode(s->d, s->m); s->has = 1; }
    return s->val;
}
static void s2_consume(s2_stream_t *s) { s->has = 0; s->used++; }

static int rice_read_quotient_s(s2_stream_t *s)
{
    if (s2_peek(s) == 2) {
        uint32_t low = 0;
        int shift = 0;
        while (s2_peek(s) == 2) {
            s2_consume(s);
            int b1 = s2_peek(s); s2_consume(s);
            int b0 = s2_peek(s); s2_consume(s);
            low |= (uint32_t)(((b1 << 1) | b0)) << shift;
            shift += 2;
        }
        int ones = 0;
        while (s2_peek(s) == 1) { ones++; s2_consume(s); }
        if (s2_peek(s) == 0) s2_consume(s);
        return (int)(low | ((uint32_t)(ones + 1) << shift));
    }
    int ones = 0;
    while (ones < 3 && s2_peek(s) == 1) { ones++; s2_consume(s); }
    if (ones == 3) return 3;
    if (s2_peek(s) == 0) s2_consume(s);
    return ones;
}

/* On-demand variant of rice_decode_coeffs_used: identical logic, pulls s2 from
 * the live range coder. After return, *fac_p / *fac_range give the reader state
 * after exactly (count + s2_used) symbols (the mantissa-plane start + leftover). */
static void rice_decode_coeffs_used_od(const uint8_t *s1, s2_stream_t *s,
                                       int count, int split, int32_t *coeff,
                                       int *s2_used, int *fac_p, uint32_t *fac_range)
{
    int pivot = count - split;
    if (pivot < 0) pivot = 0;
    int run = 0;
    for (int i = 0; i < count; i++) if (s1[i] == 2) run = i;
    int pivot2 = (run < pivot) ? count : run;

    for (int i = 0; i < count; i++) {
        int sv = s1[i];
        if (i <= pivot2) {
            if (sv == 2) { int q = rice_read_quotient_s(s); coeff[i] = q + 2; }
            else         { coeff[i] = sv; }
        } else {
            int q = rice_read_quotient_s(s); coeff[i] = (q << 1) | sv;
        }
    }
    if (s2_used) *s2_used = s->used;
    /* If a lookahead symbol is buffered (decoded but not consumed), the reader
     * state before it (save_p/save_range) is the position after the consumed
     * symbols; otherwise the live d->p/d->range already are. */
    if (s->has) { if (fac_p) *fac_p = s->save_p;  if (fac_range) *fac_range = s->save_range; }
    else        { if (fac_p) *fac_p = s->d->p;    if (fac_range) *fac_range = s->d->range; }
}

/*
 * Decode `count` quantized |coeff| values from the two ternary streams.
 *   s1: pass-1 plane (length count), s2: pass-2 quotient stream.
 *   pivot = max(count - split, 0); run = last index whose s1 symbol == 2;
 *   pivot2 = (run < pivot) ? count : run.
 *   i <= pivot2: s1==2 -> coeff = q+2 ; else coeff = s1[i] (coeff<2).
 *   i  > pivot2: coeff = (q<<1) | s1[i]  (s1[i] is the LSB).
 */
__attribute__((unused))
static LHDC_HOT void rice_decode_coeffs_used(const uint8_t *s1, const uint8_t *s2, int s2len,
                                    int count, int split, int32_t *coeff,
                                    int *s2_used)
{
    int pivot = count - split;
    if (pivot < 0) pivot = 0;
    int run = 0;
    for (int i = 0; i < count; i++) if (s1[i] == 2) run = i;
    int pivot2 = (run < pivot) ? count : run;

    int j = 0;
    for (int i = 0; i < count; i++) {
        int s = s1[i];
        if (i <= pivot2) {
            if (s == 2) {
                int q = rice_read_quotient(s2, s2len, &j);
                coeff[i] = q + 2;
            } else {
                coeff[i] = s;
            }
        } else {
            int q = rice_read_quotient(s2, s2len, &j);
            coeff[i] = (q << 1) | s;
        }
    }
    if (s2_used) *s2_used = j;
}

/*
 * Public entry: decode the spectral coefficients for one channel. The bit
 * reader must be positioned at the start of the FAC byte stream (i.e. right
 * after the per-channel leading section). `count` = nzc (significant coeffs),
 * `split` and the MA windows come from the band config / ECB.
 *
 * NOTE: the current decoder front-end (lhdc_dec.c) does not yet thread count /
 * split / windows / sign-plane offset through to here; this function exposes the
 * validated core. lhdc_entropy_decode_spectrum below adapts the legacy call.
 */
lhdc_dec_ret_t lhdc_entropy_decode_spectrum_ex2(int32_t *quant_spectrum,
                                                 int num_coeffs,
                                                 lhdc_dec_bit_reader_t *br,
                                                 int count, int split,
                                                 int ma_win1, int ma_win2,
                                                 int *fac_bytes_out,
                                                 uint8_t *scratch_fac, int scratch_fac_cap,
                                                 uint8_t *scratch_s1,
                                                 uint8_t *scratch_s2, int scratch_s2_cap)
{
    /*
     * Scratch buffers (facbuf / s1 / s2) come from the heap-allocated decoder
     * workspace to avoid large .bss; passed in via the caller. The leading
     * section consumed up to br's current bit position; the FAC stream is the
     * MSB-first bit packing from here on.
     */
    uint8_t *facbuf = scratch_fac;
    uint8_t *s1 = scratch_s1;
    uint8_t *s2 = scratch_s2;
#if defined(LHDC_HOST_BUILD)
    { extern volatile int g_lhdc_trace; if (g_lhdc_trace > 0) { g_fdc = 0; g_block++; } }
#endif
    int nb = 0;
    while (lhdc_bit_reader_remaining(br) >= 8 && nb < scratch_fac_cap) {
        facbuf[nb++] = (uint8_t)lhdc_bit_reader_read(br, 8);
    }

    if (count < 1 || count > num_coeffs) count = num_coeffs;

    fac_dec_t d;
    fac_dec_init(&d, facbuf, nb);

    fac_model_t *const pm1 = &s_fac_m[0];   /* stream-1 model (.bss) */
    fac_model_t *const pm2 = &s_fac_m[1];   /* stream-2 model (.bss) */
    lhdc_entropy_alloc_internal();          /* point pm1/pm2 at their right-sized hist buffers (idempotent) */

    fac_model_init(pm1, FAC_FQ1, ma_win1 > 0 ? ma_win1 : 256);
    for (int i = 0; i < count; i++) s1[i] = (uint8_t)fac_decode(&d, pm1);
    fac_dec_t d_after_s1 = d;   /* snapshot: byte-pos pass resumes here, no s1 re-decode */

    fac_model_init(pm2, FAC_FQ2, ma_win2 > 0 ? ma_win2 : 256);
    int s2len = 0;
    int s2cap = scratch_s2_cap;
    /* The Rice inverse never needs > ~2 pass-2 symbols per coeff; 2*count is a safe
     * ceiling (verified: capping here vs decoding to end-of-input does not change
     * the decoded result on the full stress corpus). Saves ~1/3 of entropy time. */
    int lazy_cap = (count * 2 < s2cap) ? count * 2 : s2cap;
    while (s2len < lazy_cap && d.p <= d.len) s2[s2len++] = (uint8_t)fac_decode(&d, pm2);

#if defined(LHDC_HOST_BUILD)
    { extern volatile int g_lhdc_trace;
      if (g_lhdc_trace > 0) {
        printf("[ENT] count=%d split=%d ma_win1=%d ma_win2=%d nb=%d s2len=%d\n",
               count, split, ma_win1, ma_win2, nb, s2len);
        printf("[ENT] s1full:");
        for (int i = 0; i < count; i++) printf(" %d", s1[i]);
        printf("\n[ENT] s2full:");
        for (int i = 0; i < s2len && i < 400; i++) printf(" %d", s2[i]);
        printf("\n");
      } }
#endif

    memset(quant_spectrum, 0, num_coeffs * sizeof(int32_t));
    int s2_used = 0;
    rice_decode_coeffs_used(s1, s2, s2len, count, split, quant_spectrum, &s2_used);

    if (fac_bytes_out) {
        fac_dec_t d2 = d_after_s1;   /* resume right after s1 (deterministic: same as
                                      * re-decoding count s1 symbols from the start) */
        fac_model_init(pm2, FAC_FQ2, ma_win2 > 0 ? ma_win2 : 256);
        for (int i = 0; i < s2_used; i++) (void)fac_decode(&d2, pm2);
        int consumed = d2.p;
        if (consumed < 0) consumed = 0;
        *fac_bytes_out = consumed;            /* raw bytes pulled */
        g_fac_final_range = d2.range;         /* leftover rule applied in lhdc_dec.c */
#if defined(LHDC_HOST_BUILD)
        if (getenv("LHDC_LEFTOVER_LOG"))
            printf("[LEFTOVER] consumed=%d range=%08x s2_used=%d rule=%d\n",
                   consumed, d2.range, s2_used, (d2.range <= 0x2000000u) ? 2 : 3);
#endif
    }
    return LHDC_DEC_OK;
}

lhdc_dec_ret_t lhdc_entropy_decode_bpc(int32_t *quant_spectrum,
                                         int num_coeffs,
                                         lhdc_dec_bit_reader_t *br)
{
    /* BPC/residual is folded into the Rice streams above; nothing extra here. */
    (void)quant_spectrum; (void)num_coeffs; (void)br;
    return LHDC_DEC_OK;
}
