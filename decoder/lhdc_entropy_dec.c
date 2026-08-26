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

/* Inline control for the range coder. fac_decode + fac_model_update run once per
 * decoded symbol (~1M/s at 192k/1000k), so they must fold into the caller's loop:
 * an out-of-line Xtensa windowed call (entry/retw + register window rotation)
 * costs more than the symbol decode itself. fac_model_rescale is the opposite --
 * it fires roughly once per 20 symbols, and inlining its three call sites is what
 * pushed fac_decode past GCC's size heuristic and out of line in the first place,
 * so it is pinned out-of-line (still IRAM). */
#define LHDC_ALWAYS_INLINE __attribute__((always_inline)) inline
#define LHDC_NOINLINE      __attribute__((noinline))


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

/* The FAC alphabet is always ternary (m->n is only ever assigned FAC_NSYM), so
 * the model loops are written out for n == 3: no trip counts, no m->n reload,
 * and the cumulative freqs stay in registers.
 *
 * The 64-bit product is also gone. cum[i] <= total and scale = 0x80000000/total
 * (floor), so cum[i]*scale <= total*floor(0x80000000/total) <= 0x80000000 --
 * it cannot overflow 32 bits. Verified by differential test over 20M decoded
 * symbols (max product observed: exactly 0x80000000). On Xtensa that turns a
 * mull/muluh/shift sequence into a single mull per entry. */
_Static_assert(FAC_NSYM == 3, "fac model fast paths assume a ternary alphabet");

static LHDC_HOT LHDC_NOINLINE void fac_model_rescale(fac_model_t *m)
{
    const uint32_t f0 = m->freq[0], f1 = m->freq[1], f2 = m->freq[2];
    m->thresh[0] = f0 + (f0 >> 5);
    m->thresh[1] = f1 + (f1 >> 5);
    m->thresh[2] = f2 + (f2 >> 5);

    const uint32_t c1 = f0, c2 = f0 + f1, c3 = c2 + f2;   /* running cumulative */
    const uint32_t scale = 0x80000000u / c3;              /* c3 = total > 0 */
    m->cum[0] = 0;
    m->cum[1] = (c1 * scale) >> 16;
    m->cum[2] = (c2 * scale) >> 16;
    m->cum[3] = (c3 * scale) >> 16;
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

/* Same state transition as before, with the history write hoisted above the
 * freq update: fac_model_rescale touches neither hist nor pos/count, so the
 * reorder is observationally identical (checked symbol-by-symbol, including the
 * full history buffer, against the previous version over 20M symbols) and it
 * frees the compiler to keep `pos` in a register across the branch. */
static LHDC_HOT LHDC_ALWAYS_INLINE void fac_model_update(fac_model_t *m, int sym)
{
    uint32_t *const freq = m->freq;
    if (m->count < m->window) {
        m->hist[m->count] = (uint8_t)sym;
        m->count++;
        if (++freq[sym] >= m->thresh[sym]) fac_model_rescale(m);
    } else {
        int pos = m->pos;
        const int old = m->hist[pos];
        if (freq[old] >= 2) freq[old]--;
        m->hist[pos] = (uint8_t)sym;
        if (++pos >= m->window) pos = 0;
        m->pos = pos;
        if (++freq[sym] >= m->thresh[sym]) fac_model_rescale(m);
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

static LHDC_HOT LHDC_ALWAYS_INLINE uint8_t fac_byte(fac_dec_t *d)
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

static LHDC_HOT LHDC_ALWAYS_INLINE int fac_decode(fac_dec_t *d, fac_model_t *m)
{
#if defined(LHDC_HOST_BUILD)
    extern volatile int g_lhdc_trace; extern int g_fdc;   /* reset per-channel by the entropy fn */
    uint32_t dbg_range=d->range, dbg_code=d->code;
    uint32_t dbg_cum1=m->cum[1], dbg_cum2=m->cum[2], dbg_f0=m->freq[0],dbg_f1=m->freq[1],dbg_f2=m->freq[2],dbg_cnt=m->count;
#endif
    /* Divide-free, branch-flat ternary symbol search. The ARM reference computed
     * `target = code / r` per symbol; Xtensa LX6's divide is a ~30-cycle
     * non-pipelined instruction whereas its 32-bit multiply is single-cycle, so
     * use the integer identity floor(code/r) >= c  <=>  code >= r*c (c>=0, r>0)
     * and compare r*cum[] against code directly. Bit-IDENTICAL to the old
     * target+clamp path: the `target >= FAC_TOTAL -> FAC_TOTAL-1` clamp is
     * subsumed by stopping at the last symbol. No overflow: r*cum[k] <=
     * r*FAC_TOTAL = (range>>15)<<15 <= range < 2^32.
     *
     * Written out for the ternary alphabet instead of looping on m->n: that
     * drops the trip-count reload and the loop branch, and cum[0] is always 0
     * so symbol 0 needs no subtract at all. The decoder state is pulled into
     * locals for the duration so the renormalize loop works on registers
     * instead of re-reading the struct through a pointer each iteration. */
    uint32_t range = d->range, code = d->code;
    const uint32_t r = range >> FAC_TOTAL_BITS;
    const uint32_t c1 = m->cum[1];
    uint32_t lo, hi;
    int sym;
    if (r * c1 > code) {
        sym = 0; lo = 0;  hi = c1;
    } else {
        const uint32_t c2 = m->cum[2];
        if (r * c2 > code) { sym = 1; lo = c1; hi = c2; }
        else               { sym = 2; lo = c2; hi = m->cum[3]; }
    }
    code  -= r * lo;
    range  = r * (hi - lo);
    if (range < (1u << 24)) {                 /* renormalize (was `(range>>24)==0`) */
        const uint8_t *const data = d->data;
        int p = d->p;
        const int len = d->len;
        do {
            code = (code << 8) | ((p < len) ? data[p] : 0);
            p++;
            range <<= 8;
        } while (range < (1u << 24));
        d->p = p;
    }
    d->range = range;
    d->code = code;
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

/*
 * Lazily-decoded pass-2 symbol stream.
 *
 * The old flow decoded the pass-2 stream THREE times per channel:
 *   1. an eager fill of min(2*count, cap) symbols  (1920 at 192k),
 *   2. the Rice inverse, which consumes only `s2_used` of them (typically a
 *      small fraction),
 *   3. a full re-decode of s2_used symbols from a re-initialized model, purely
 *      to recover the byte position where the mantissa plane starts.
 * At 192k that is 960 (s1) + 1920 (eager) + s2_used symbols per channel, of
 * which the 1920 are mostly thrown away. The range coder is serial and is the
 * single largest CPU consumer in the decoder, so this dominated the 192k
 * budget on a chip without ARM's headroom.
 *
 * s2_at() decodes symbols on demand into the SAME array, one at a time, under
 * the SAME guard the eager loop used (`len < cap && d->p <= d->len`). That
 * makes the produced prefix identical to the old eager fill symbol-for-symbol,
 * and s2_at() returns -1 exactly where the old code would have read past
 * `s2len` -- so the Rice inverse below is a literal transcription of the array
 * version with `j < n && s2[j]` rewritten as `s2_at(j)`.
 *
 * save_p/save_range snapshot the decoder state BEFORE the most recently decoded
 * symbol, which is what removes pass 3: after the Rice inverse consumes
 * s2_used symbols, the state after exactly s2_used symbols is either the live
 * one (nothing was read ahead) or the snapshot (one lookahead symbol was
 * decoded but not consumed). See s2_state_after().
 */
typedef struct {
    fac_dec_t   *d;
    fac_model_t *m;
    uint8_t     *s2;         /* backing array (same scratch as before) */
    int          len;        /* symbols decoded so far */
    int          cap;        /* == the old lazy_cap */
    int          eos;        /* fill guard failed; len is final */
    int          save_idx;   /* index of the last decoded symbol */
    int          save_p;     /* d->p before that symbol */
    uint32_t     save_range; /* d->range before that symbol */
} s2_lazy_t;

/* s2[i], or -1 when i is past the end of what the stream can produce. */
static LHDC_HOT LHDC_ALWAYS_INLINE int s2_at(s2_lazy_t *z, int i)
{
    while (i >= z->len) {
        if (z->eos) return -1;
        if (z->len >= z->cap || z->d->p > z->d->len) { z->eos = 1; return -1; }
        z->save_idx   = z->len;
        z->save_p     = z->d->p;
        z->save_range = z->d->range;
        z->s2[z->len++] = (uint8_t)fac_decode(z->d, z->m);
    }
    return z->s2[i];
}

/* Decoder state (byte position + range) after exactly `used` pass-2 symbols --
 * what the old pass-3 re-decode computed. */
static void s2_state_after(s2_lazy_t *z, int used, int *p_out, uint32_t *range_out)
{
    if (used < z->len) {
        /* Only case: one lookahead symbol was decoded but not consumed. */
        *p_out = z->save_p; *range_out = z->save_range;
        (void)z->save_idx;   /* == used by construction */
    } else {
        /* The Rice inverse walked past what the stream produced (end of input).
         * The old code re-decoded `used` symbols regardless, so match it. */
        for (int i = z->len; i < used; i++) (void)fac_decode(z->d, z->m);
        *p_out = z->d->p; *range_out = z->d->range;
    }
}

static LHDC_HOT int rice_read_quotient(s2_lazy_t *z, int *jp)
{
    int j = *jp;
    if (s2_at(z, j) == 2) {
        uint32_t low = 0;
        int shift = 0;
        while (s2_at(z, j) == 2) {
            j++;
            int v1 = s2_at(z, j); int b1 = (v1 < 0) ? 0 : v1; j++;
            int v0 = s2_at(z, j); int b0 = (v0 < 0) ? 0 : v0; j++;
            low |= (uint32_t)(((b1 << 1) | b0)) << shift;
            shift += 2;
        }
        int ones = 0;
        while (s2_at(z, j) == 1) { ones++; j++; }
        if (s2_at(z, j) == 0) j++;
        *jp = j;
        return (int)(low | ((uint32_t)(ones + 1) << shift));
    }
    int ones = 0;
    while (ones < 3 && s2_at(z, j) == 1) { ones++; j++; }
    if (ones == 3) { *jp = j; return 3; }
    if (s2_at(z, j) == 0) j++;
    *jp = j;
    return ones;
}

/*
 * Decode `count` quantized |coeff| values from the two ternary streams.
 *   s1: pass-1 plane (length count), z: lazy pass-2 quotient stream.
 *   pivot = max(count - split, 0); run = last index whose s1 symbol == 2;
 *   pivot2 = (run < pivot) ? count : run.
 *   i <= pivot2: s1==2 -> coeff = q+2 ; else coeff = s1[i] (coeff<2).
 *   i  > pivot2: coeff = (q<<1) | s1[i]  (s1[i] is the LSB).
 */
static LHDC_HOT void rice_decode_coeffs_used(const uint8_t *s1, s2_lazy_t *z,
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
                int q = rice_read_quotient(z, &j);
                coeff[i] = q + 2;
            } else {
                coeff[i] = s;
            }
        } else {
            int q = rice_read_quotient(z, &j);
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

    fac_model_init(pm2, FAC_FQ2, ma_win2 > 0 ? ma_win2 : 256);
    int s2cap = scratch_s2_cap;
    /* The Rice inverse never needs > ~2 pass-2 symbols per coeff; 2*count is a safe
     * ceiling (verified: capping here vs decoding to end-of-input does not change
     * the decoded result on the full stress corpus). The symbols are now decoded
     * ON DEMAND under this same bound instead of being pre-decoded in full: the
     * Rice inverse consumes only a fraction of them, so the eager fill was the
     * decoder's single largest block of wasted work. */
    int lazy_cap = (count * 2 < s2cap) ? count * 2 : s2cap;
    s2_lazy_t z;
    z.d = &d; z.m = pm2; z.s2 = s2;
    z.len = 0; z.cap = lazy_cap; z.eos = 0;
    z.save_idx = 0; z.save_p = d.p; z.save_range = d.range;

#if defined(LHDC_HOST_BUILD)
    { extern volatile int g_lhdc_trace;
      if (g_lhdc_trace > 0) {
        printf("[ENT] count=%d split=%d ma_win1=%d ma_win2=%d nb=%d (s2 lazy)\n",
               count, split, ma_win1, ma_win2, nb);
        printf("[ENT] s1full:");
        for (int i = 0; i < count; i++) printf(" %d", s1[i]);
        printf("\n");
      } }
#endif

    memset(quant_spectrum, 0, num_coeffs * sizeof(int32_t));
    int s2_used = 0;
    rice_decode_coeffs_used(s1, &z, count, split, quant_spectrum, &s2_used);

    if (fac_bytes_out) {
        /* Byte position after exactly s2_used pass-2 symbols. The old code got
         * this by re-initializing the model and re-decoding s2_used symbols from
         * a snapshot taken after s1; the lazy stream already holds that state (or
         * the snapshot taken just before its single lookahead symbol), so the
         * whole third pass over the range coder is gone. */
        int consumed = 0;
        uint32_t final_range = 0;
        s2_state_after(&z, s2_used, &consumed, &final_range);
        if (consumed < 0) consumed = 0;
        *fac_bytes_out = consumed;            /* raw bytes pulled */
        g_fac_final_range = final_range;      /* leftover rule applied in lhdc_dec.c */
#if defined(LHDC_HOST_BUILD)
        if (getenv("LHDC_LEFTOVER_LOG"))
            printf("[LEFTOVER] consumed=%d range=%08x s2_used=%d rule=%d\n",
                   consumed, final_range, s2_used, (final_range <= 0x2000000u) ? 2 : 3);
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
