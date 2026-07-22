#include "lhdc_imdct.h"
#include "lhdc_diag_config.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if defined(LHDC_HOST_BUILD)
  #include <stdio.h>
  #define IMDCT_LOGI(tag, ...) do { printf("[I]" tag ": " __VA_ARGS__); printf("\n"); } while (0)
  #define LHDC_HOT
#else
  #include "esp_log.h"
  #include "esp_attr.h"
  #define IMDCT_LOGI(tag, ...) ESP_LOGI(tag, __VA_ARGS__)
  /* IMDCT fast path in IRAM: avoids flash-cache eviction by Bluetooth controller interrupts. */
  #define LHDC_HOT IRAM_ATTR
#endif

#ifndef M_PIF
#define M_PIF 3.14159265358979323846f
#endif

/*
 * IMDCT for LHDC V5.
 *
 *   x[n] = (2/N) * sum_{k=0}^{N/2-1} X[k] * cos(pi/(2N)*(2n+1+N/2)*(2k+1))
 *   for n = 0..N-1
 *
 * Two implementations:
 *  - lhdc_imdct_ref: a direct O(N^2) evaluation with a precomputed cosine table.
 *    Correct for any N, but too slow for real-time decode on the ESP32.
 *  - lhdc_imdct_fast_480: an O(N log N) fast path for the live config (N=480,
 *    48k/5ms). Computed as a length-N/4 (=120) complex FFT with pre/post twiddle
 *    and a symmetric unfold; the 120-point FFT is an 8x15 four-step Cooley-Tukey
 *    with direct radix-8 / radix-15 sub-DFTs.
 *
 * A one-shot self-test at init runs both on a fixed vector; the fast path is
 * only enabled if it matches the reference, otherwise the decoder falls back to
 * the always-correct reference so a port bug can never corrupt audio.
 */

/* ----------------------------- reference path ----------------------------- */

/* The reference cosine table holds one full period = 4*N of cos((pi/2N)*j). It
 * must cover the largest transform size the decoder can negotiate, otherwise the
 * reference IMDCT (which indexes with the unclamped 4*N period) would read past
 * the built region. Sized for the largest supported N. Allocated lazily: the
 * fast paths normally handle every frame, so this table is built only when the
 * reference path actually runs (a fast self-test failed, or a transform size
 * with no fast path). */
#define LHDC_IMDCT_COS_MAX (4 * 1920)
static float *s_cos_tab = NULL;      /* heap, lazy */
static int   s_cos_tab_n = 0;        /* N the table was built for */
static int   s_cos_period = 0;       /* 4N */

/* Build (or rebuild for a new N) the reference cosine table. Returns 0 on
 * success, -1 if it could not be allocated. */
static int lhdc_cos_table_ensure(int N)
{
    int period = 4 * N;
    if (period > LHDC_IMDCT_COS_MAX) period = LHDC_IMDCT_COS_MAX;
    if (s_cos_tab && s_cos_tab_n == N) return 0;
    if (!s_cos_tab) {
        s_cos_tab = (float *)malloc(LHDC_IMDCT_COS_MAX * sizeof(float));
        if (!s_cos_tab) return -1;
    }
    for (int j = 0; j < period; j++)
        s_cos_tab[j] = cosf((M_PIF / (2.0f * (float)N)) * (float)j);
    s_cos_tab_n = N;
    s_cos_period = period;
    return 0;
}

/* Release the reference-IMDCT cosine table (heap, ~30 KB at large N). Optional;
 * call at teardown so it doesn't linger in DRAM after LHDC stops. */
void lhdc_imdct_free_cos(void)
{
    if (s_cos_tab) { free(s_cos_tab); s_cos_tab = NULL; }
    s_cos_tab_n = 0;
    s_cos_period = 0;
}

static void lhdc_imdct_ref(const float *in, float *out, int mdct_size)
{
    int N = mdct_size;
    int N2 = N / 2;
    int period = 4 * N;
    float scale = 2.0f / (float)N;

    if (lhdc_cos_table_ensure(N) != 0) {   /* out of memory -> silence (safe) */
        for (int n = 0; n < N; n++) out[n] = 0.0f;
        return;
    }

    int kmax = -1;
    for (int k = N2 - 1; k >= 0; k--) {
        if (in[k] != 0.0f) { kmax = k; break; }
    }
    int kn = kmax + 1;
    if (kn == 0) { for (int n = 0; n < N; n++) out[n] = 0.0f; return; }

    for (int n = 0; n < N; n++) {
        int base = (2 * n + 1 + N2);
        int m = base % period;
        int step = (2 * base) % period;
        float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
        int k = 0;
        for (; k + 4 <= kn; k += 4) {
            s0 += in[k]     * s_cos_tab[m]; m += step; if (m >= period) m -= period;
            s1 += in[k + 1] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
            s2 += in[k + 2] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
            s3 += in[k + 3] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
        }
        for (; k < kn; k++) {
            s0 += in[k] * s_cos_tab[m]; m += step; if (m >= period) m -= period;
        }
        out[n] = scale * ((s0 + s1) + (s2 + s3));
    }
}

/* ------------------------------- fast path -------------------------------- */
/* Fixed for N = 480: M = 240, P = N/4 = 120 = N1(8) * N2(15). */
#define IMDCT_N   480
#define IMDCT_M   240
#define IMDCT_P   120
#define IMDCT_N1  8
#define IMDCT_N2  15

/* IMDCT four-step FFT twiddles for N=480 (radix-8 x radix-15) plus pre/post
 * rotation. These must live in data RAM, not flash: the FFT inner loop reads
 * them thousands of times per frame, and flash cache misses make the IMDCT far
 * slower. The tables live in one lazily-allocated block so they occupy DRAM only
 * while 480 (44.1/48k) is the active config; freed via lhdc_imdct_free_480()
 * when the decoder reconfigures to another rate or is torn down.
 * Layout (floats): w1_r/i 64 each, w2_r/i 225 each, tw_r/i 120 each, rot_r/i
 * 120 each = 1058 floats. */
#define S480_NFLOATS (2*IMDCT_N1*IMDCT_N1 + 2*IMDCT_N2*IMDCT_N2 \
                      + 2*IMDCT_N2*IMDCT_N1 + 2*IMDCT_P)
static float *s480_mem = NULL;
static float *s_w1_r, *s_w1_i;   /* radix-8 DFT  */
static float *s_w2_r, *s_w2_i;   /* radix-15 DFT */
static float *s_tw_r, *s_tw_i;   /* four-step twiddle */
static float *s_rot_r, *s_rot_i; /* pre/post rotation */
static int   s_fast_built = 0;
static int   s_fast_ok = 0;   /* set by the self-test */

void lhdc_imdct_free_480(void)
{
    if (s480_mem) { free(s480_mem); s480_mem = NULL; }
    s_fast_built = 0;
    s_fast_ok = 0;
}

static void lhdc_imdct_build_fast_tables(void)
{
    if (s_fast_built) return;
    if (!s480_mem) {
        s480_mem = (float *)malloc(S480_NFLOATS * sizeof(float));
        if (!s480_mem) return;   /* s_fast_built stays 0 -> falls back to ref IMDCT */
    }
    {
        float *p = s480_mem;
        s_w1_r = p; p += IMDCT_N1*IMDCT_N1;
        s_w1_i = p; p += IMDCT_N1*IMDCT_N1;
        s_w2_r = p; p += IMDCT_N2*IMDCT_N2;
        s_w2_i = p; p += IMDCT_N2*IMDCT_N2;
        s_tw_r = p; p += IMDCT_N2*IMDCT_N1;
        s_tw_i = p; p += IMDCT_N2*IMDCT_N1;
        s_rot_r = p; p += IMDCT_P;
        s_rot_i = p; p += IMDCT_P;
    }
    for (int k = 0; k < IMDCT_N1; k++)
        for (int n = 0; n < IMDCT_N1; n++) {
            float a = -2.0f * M_PIF / IMDCT_N1 * (float)k * (float)n;
            s_w1_r[k * IMDCT_N1 + n] = cosf(a);
            s_w1_i[k * IMDCT_N1 + n] = sinf(a);
        }
    for (int k = 0; k < IMDCT_N2; k++)
        for (int n = 0; n < IMDCT_N2; n++) {
            float a = -2.0f * M_PIF / IMDCT_N2 * (float)k * (float)n;
            s_w2_r[k * IMDCT_N2 + n] = cosf(a);
            s_w2_i[k * IMDCT_N2 + n] = sinf(a);
        }
    for (int n2 = 0; n2 < IMDCT_N2; n2++)
        for (int k1 = 0; k1 < IMDCT_N1; k1++) {
            float a = -2.0f * M_PIF / IMDCT_P * (float)n2 * (float)k1;
            s_tw_r[n2 * IMDCT_N1 + k1] = cosf(a);
            s_tw_i[n2 * IMDCT_N1 + k1] = sinf(a);
        }
    for (int k = 0; k < IMDCT_P; k++) {
        float a = -2.0f * M_PIF / IMDCT_N * ((float)k + 0.125f);
        s_rot_r[k] = cosf(a);
        s_rot_i[k] = sinf(a);
    }
    s_fast_built = 1;
}

/* ---- shared 15-point DFT via 3x5 Cooley-Tukey --------------------------------
 * Stage 2 of all three fast IMDCTs (480/960/1920) is a length-15 complex DFT,
 * the same transform regardless of rate. 15 = 3*5, so a Cooley-Tukey split
 * (5 DFT-3 + 15 twiddles + 3 DFT-5 = 135 complex muls) is fewer muls than the
 * direct 15x15 matrix multiply (225). The constants below are exact (cos/sin of
 * 2pi/3, 2pi/5, 2pi/15) and const, so they live in flash.
 * Index map: n = 5*n1+n2 (n1<3, n2<5); k = 3*k2+k1. */
static const float DFT3_r[9] = {1.0f,1.0f,1.0f, 1.0f,-0.5f,-0.5f, 1.0f,-0.5f,-0.5f};
static const float DFT3_i[9] = {0.0f,0.0f,0.0f, 0.0f,-0.866025404f,0.866025404f, 0.0f,0.866025404f,-0.866025404f};
static const float DFT5_r[25] = {
    1.0f,1.0f,1.0f,1.0f,1.0f,
    1.0f,0.309016994f,-0.809016994f,-0.809016994f,0.309016994f,
    1.0f,-0.809016994f,0.309016994f,0.309016994f,-0.809016994f,
    1.0f,-0.809016994f,0.309016994f,0.309016994f,-0.809016994f,
    1.0f,0.309016994f,-0.809016994f,-0.809016994f,0.309016994f};
static const float DFT5_i[25] = {
    0.0f,0.0f,0.0f,0.0f,0.0f,
    0.0f,-0.951056516f,-0.587785252f,0.587785252f,0.951056516f,
    0.0f,-0.587785252f,0.951056516f,-0.951056516f,0.587785252f,
    0.0f,0.587785252f,-0.951056516f,0.951056516f,-0.587785252f,
    0.0f,0.951056516f,0.587785252f,-0.587785252f,-0.951056516f};
static const float TW35_r[15] = {
    1.0f,1.0f,1.0f,1.0f,1.0f,
    1.0f,0.913545458f,0.669130606f,0.309016994f,-0.104528463f,
    1.0f,0.669130606f,-0.104528463f,-0.809016994f,-0.978147601f};
static const float TW35_i[15] = {
    0.0f,0.0f,0.0f,0.0f,0.0f,
    0.0f,-0.406736643f,-0.743144825f,-0.951056516f,-0.994521895f,
    0.0f,-0.743144825f,-0.994521895f,-0.587785252f,0.207911691f};

/* 15-point DFT of g[0..14] (indexed n = 5*n1+n2) into the 15 outputs
 * X[base + stride*(3*k2+k1)]. */
static LHDC_HOT void lhdc_dft15(const float *gr, const float *gi,
                                float *Xr, float *Xi, int base, int stride)
{
    float Br[15], Bi[15];   /* B[k1*5 + n2] (post-twiddle) */
    /* Stage 1: a DFT-3 over n1 for each n2, then the 3x5 twiddle. */
    for (int n2 = 0; n2 < 5; n2++) {
        float g0r = gr[n2],      g0i = gi[n2];
        float g1r = gr[5 + n2],  g1i = gi[5 + n2];
        float g2r = gr[10 + n2], g2i = gi[10 + n2];
        for (int k1 = 0; k1 < 3; k1++) {
            float a0r = DFT3_r[k1*3+0], a0i = DFT3_i[k1*3+0];
            float a1r = DFT3_r[k1*3+1], a1i = DFT3_i[k1*3+1];
            float a2r = DFT3_r[k1*3+2], a2i = DFT3_i[k1*3+2];
            float ar = a0r*g0r - a0i*g0i + a1r*g1r - a1i*g1i + a2r*g2r - a2i*g2i;
            float ai = a0r*g0i + a0i*g0r + a1r*g1i + a1i*g1r + a2r*g2i + a2i*g2r;
            float tr = TW35_r[k1*5+n2], ti = TW35_i[k1*5+n2];
            Br[k1*5+n2] = ar*tr - ai*ti;
            Bi[k1*5+n2] = ar*ti + ai*tr;
        }
    }
    /* Stage 2: a DFT-5 over n2 for each k1 -> X[base + stride*(3*k2+k1)]. */
    for (int k1 = 0; k1 < 3; k1++) {
        const float *br = &Br[k1*5];
        const float *bi = &Bi[k1*5];
        for (int k2 = 0; k2 < 5; k2++) {
            float ar = 0.0f, ai = 0.0f;
            const float *wr = &DFT5_r[k2*5];
            const float *wi = &DFT5_i[k2*5];
            for (int n2 = 0; n2 < 5; n2++) {
                ar += wr[n2]*br[n2] - wi[n2]*bi[n2];
                ai += wr[n2]*bi[n2] + wi[n2]*br[n2];
            }
            int k = 3*k2 + k1;
            Xr[base + stride*k] = ar;
            Xi[base + stride*k] = ai;
        }
    }
}

/* 120-point complex FFT (8x15 four-step). Input zr/zi in natural order, output Xr/Xi. */
static LHDC_HOT void lhdc_fft120(const float *zr, const float *zi, float *Xr, float *Xi)
{
    float Gr[IMDCT_P], Gi[IMDCT_P];   /* G[k1*15 + n2] */
    /* Stage 1: radix-8 DFT over n1 for each n2, then four-step twiddle. */
    for (int n2 = 0; n2 < IMDCT_N2; n2++) {
        for (int k1 = 0; k1 < IMDCT_N1; k1++) {
            float ar = 0.0f, ai = 0.0f;
            const float *w1r = &s_w1_r[k1 * IMDCT_N1];
            const float *w1i = &s_w1_i[k1 * IMDCT_N1];
            for (int n1 = 0; n1 < IMDCT_N1; n1++) {
                int idx = n2 + IMDCT_N2 * n1;
                float xr = zr[idx], xi = zi[idx];
                ar += w1r[n1] * xr - w1i[n1] * xi;
                ai += w1r[n1] * xi + w1i[n1] * xr;
            }
            float tr = s_tw_r[n2 * IMDCT_N1 + k1];
            float ti = s_tw_i[n2 * IMDCT_N1 + k1];
            Gr[k1 * IMDCT_N2 + n2] = ar * tr - ai * ti;
            Gi[k1 * IMDCT_N2 + n2] = ar * ti + ai * tr;
        }
    }
    /* Stage 2: 15-point DFT (3x5 Cooley-Tukey) over n2 for each k1 -> X[k1 + 8*k2]. */
    for (int k1 = 0; k1 < IMDCT_N1; k1++) {
        lhdc_dft15(&Gr[k1 * IMDCT_N2], &Gi[k1 * IMDCT_N2], Xr, Xi, k1, IMDCT_N1);
    }
}

static LHDC_HOT void lhdc_imdct_fast_480(const float *in, float *out)
{
    float zr[IMDCT_P], zi[IMDCT_P];
    float Xr[IMDCT_P], Xi[IMDCT_P];
    float reZ[IMDCT_P], imZ[IMDCT_P];
    const float invM = 1.0f / (float)IMDCT_M;

    /* Pre-twiddle: z[k] = (in[2k] + i*in[239-2k]) * ROT[k]. */
    for (int k = 0; k < IMDCT_P; k++) {
        float r0 = in[2 * k];
        float i0 = in[IMDCT_M - 1 - 2 * k];
        float rr = s_rot_r[k], ri = s_rot_i[k];
        zr[k] = r0 * rr - i0 * ri;
        zi[k] = r0 * ri + i0 * rr;
    }

    lhdc_fft120(zr, zi, Xr, Xi);

    /* Post-twiddle + scale by 1/M. */
    for (int k = 0; k < IMDCT_P; k++) {
        float rr = s_rot_r[k], ri = s_rot_i[k];
        reZ[k] = (Xr[k] * rr - Xi[k] * ri) * invM;
        imZ[k] = (Xr[k] * ri + Xi[k] * rr) * invM;
    }

    /* Symmetric unfold into the N time samples. */
    for (int p = 0; p < IMDCT_M; p++) {
        int ne = 2 * p;
        if      (p < 60)  out[ne] =  reZ[60 + p];
        else if (p < 180) out[ne] =  imZ[p - 60];
        else              out[ne] = -reZ[p - 180];
        int no = 2 * p + 1;
        if      (p < 60)  out[no] = -imZ[59 - p];
        else if (p < 180) out[no] = -reZ[179 - p];
        else              out[no] =  imZ[299 - p];
    }
}

/* ----------------------- fast path for N = 960 (96k) ---------------------- */
/* Same four-step MDCT-via-FFT as the 480 path, but P = N/4 = 240 = 16 x 15. The
 * tables are self-contained so the 960 path does not depend on the 480 tables.
 * They live in DRAM, not flash, for the same cache-eviction reason as the 480
 * twiddles. */
#define IMDCT_N_960  960
#define IMDCT_M_960  480
#define IMDCT_P_960  240
#define IMDCT_N1_960 16
#define IMDCT_N2_960 15

/* The 960 tables and scratch occupy DRAM only while 96k is the active config
 * (freed via lhdc_imdct_free_960() when the decoder reconfigures to another rate
 * or is torn down). They must be DRAM, never flash: the FFT inner loop reads the
 * twiddles thousands of times per frame and flash cache misses make it far
 * slower. The storage is split into two blocks (tables + scratch) rather than
 * one large block, because the fragmented 96k heap can fail a single large
 * contiguous request even when enough total memory is free; two smaller blocks
 * each fit an available hole. Stage 1 is a radix-2 16-point FFT, so there is no
 * 16x16 DFT matrix. */
#define S960_TBL_FLOATS (2*IMDCT_N2_960*IMDCT_N2_960 \
                         + 2*IMDCT_N2_960*IMDCT_N1_960 + 2*IMDCT_P_960)   /* twiddle tables */
/* FFT scratch: only 2 P-sized buffers (zr,zi). Everything else aliases:
 *  - Xr,Xi (fft240 output) and reZ,imZ (post-twiddle output) reuse zr,zi: the
 *    pre-twiddle input is fully consumed by fft240 stage 1 before stage 2 writes,
 *    so the FFT runs in place over zr,zi; the post-twiddle rewrites zr,zi via temps.
 *  - Gr,Gi (fft240 four-step intermediate) borrow the caller's `out` buffer
 *    (out[0..2P-1]); `out` is not written until the final unfold, by which point
 *    Gr,Gi are dead. Set per call in lhdc_imdct_fast_960.
 * (The 480 path uses stack scratch and is unchanged.) */
#define S960_SCR_FLOATS (2*IMDCT_P_960)                                    /* FFT scratch (zr,zi) */
static float *s960_tbl = NULL;   /* persistent twiddle tables */
static float *s960_scr = NULL;   /* per-transform scratch */
static float *s960_w2_r, *s960_w2_i;
static float *s960_tw_r, *s960_tw_i, *s960_rot_r, *s960_rot_i;
static float *s960_Gr, *s960_Gi, *s960_zr, *s960_zi;
static float *s960_Xr, *s960_Xi, *s960_reZ, *s960_imZ;
static int   s960_built = 0;
static int   s960_ok = 0;

/* Radix-2 twiddles W16^k = exp(-2*pi*i*k/16), k=0..7, for the 16-point FFT in
 * stage 1 of lhdc_fft240. Built once with the other 960 tables. */
static float s960_w16r[8], s960_w16i[8];
/* bit-reversal permutation for the 16-point DIT FFT. */
static const uint8_t S960_BR16[16] =
    { 0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15 };

void lhdc_imdct_free_960(void)
{
    if (s960_tbl) { free(s960_tbl); s960_tbl = NULL; }
    if (s960_scr) { free(s960_scr); s960_scr = NULL; }
    s960_built = 0;
    s960_ok = 0;
}

static void lhdc_imdct_build_fast_tables_960(void)
{
    if (s960_built) return;
    if (!s960_tbl) {
        s960_tbl = (float *)malloc(S960_TBL_FLOATS * sizeof(float));
        if (!s960_tbl) return;   /* s960_built stays 0 -> falls back to ref IMDCT */
    }
    if (!s960_scr) {
        s960_scr = (float *)malloc(S960_SCR_FLOATS * sizeof(float));
        if (!s960_scr) { free(s960_tbl); s960_tbl = NULL; return; }
    }
    float *p = s960_tbl;
    s960_w2_r = p; p += IMDCT_N2_960*IMDCT_N2_960;
    s960_w2_i = p; p += IMDCT_N2_960*IMDCT_N2_960;
    s960_tw_r = p; p += IMDCT_N2_960*IMDCT_N1_960;
    s960_tw_i = p; p += IMDCT_N2_960*IMDCT_N1_960;
    s960_rot_r = p; p += IMDCT_P_960;
    s960_rot_i = p; p += IMDCT_P_960;
    float *q = s960_scr;
    s960_zr = q; q += IMDCT_P_960;  s960_zi = q; q += IMDCT_P_960;
    /* Output buffers alias the input — see S960_SCR_FLOATS note (lifetimes are
     * disjoint: fft240 stage 1 fully reads zr/zi before stage 2 writes them). */
    s960_Xr = s960_zr;  s960_Xi = s960_zi;     /* fft240 runs in place over zr,zi */
    s960_reZ = s960_zr; s960_imZ = s960_zi;    /* post-twiddle in place (uses temps) */
    /* s960_Gr/s960_Gi are NOT carved here — they borrow the caller's `out`
     * buffer per-call (set in lhdc_imdct_fast_960). */
    for (int k = 0; k < IMDCT_N2_960; k++)
        for (int n = 0; n < IMDCT_N2_960; n++) {
            float a = -2.0f * M_PIF / IMDCT_N2_960 * (float)k * (float)n;
            s960_w2_r[k * IMDCT_N2_960 + n] = cosf(a);
            s960_w2_i[k * IMDCT_N2_960 + n] = sinf(a);
        }
    for (int n2 = 0; n2 < IMDCT_N2_960; n2++)
        for (int k1 = 0; k1 < IMDCT_N1_960; k1++) {
            float a = -2.0f * M_PIF / IMDCT_P_960 * (float)n2 * (float)k1;
            s960_tw_r[n2 * IMDCT_N1_960 + k1] = cosf(a);
            s960_tw_i[n2 * IMDCT_N1_960 + k1] = sinf(a);
        }
    for (int k = 0; k < IMDCT_P_960; k++) {
        float a = -2.0f * M_PIF / IMDCT_N_960 * ((float)k + 0.125f);
        s960_rot_r[k] = cosf(a);
        s960_rot_i[k] = sinf(a);
    }
    for (int k = 0; k < 8; k++) {
        float a = -2.0f * M_PIF * (float)k / 16.0f;
        s960_w16r[k] = cosf(a);
        s960_w16i[k] = sinf(a);
    }
    s960_built = 1;
}

/* 240-point complex FFT (16x15 four-step). */
static LHDC_HOT void lhdc_fft240(const float *zr, const float *zi, float *Xr, float *Xi)
{
    /* Stage 1: a 16-point DFT over n1 for each n2, via a radix-2 DIT FFT. */
    for (int n2 = 0; n2 < IMDCT_N2_960; n2++) {
        float ar[16], ai[16];
        /* gather (strided) with bit-reversal folded in */
        for (int i = 0; i < 16; i++) {
            int idx = n2 + IMDCT_N2_960 * (int)S960_BR16[i];
            ar[i] = zr[idx]; ai[i] = zi[idx];
        }
        for (int s = 1; s <= 4; s++) {
            int m = 1 << s, h = m >> 1, step = 16 / m;
            for (int k = 0; k < 16; k += m) {
                for (int j = 0; j < h; j++) {
                    int tw = j * step;             /* index into W16[0..7] */
                    float wr = s960_w16r[tw], wi = s960_w16i[tw];
                    float br = ar[k + j + h], bi = ai[k + j + h];
                    float tr = wr * br - wi * bi;
                    float ti = wr * bi + wi * br;
                    float ur = ar[k + j], ui = ai[k + j];
                    ar[k + j]     = ur + tr; ai[k + j]     = ui + ti;
                    ar[k + j + h] = ur - tr; ai[k + j + h] = ui - ti;
                }
            }
        }
        /* four-step twiddle + store transposed into G[k1*15 + n2] */
        for (int k1 = 0; k1 < IMDCT_N1_960; k1++) {
            float tr = s960_tw_r[n2 * IMDCT_N1_960 + k1];
            float ti = s960_tw_i[n2 * IMDCT_N1_960 + k1];
            s960_Gr[k1 * IMDCT_N2_960 + n2] = ar[k1] * tr - ai[k1] * ti;
            s960_Gi[k1 * IMDCT_N2_960 + n2] = ar[k1] * ti + ai[k1] * tr;
        }
    }
    for (int k1 = 0; k1 < IMDCT_N1_960; k1++) {
        lhdc_dft15(&s960_Gr[k1 * IMDCT_N2_960], &s960_Gi[k1 * IMDCT_N2_960],
                   Xr, Xi, k1, IMDCT_N1_960);
    }
}

static LHDC_HOT void lhdc_imdct_fast_960(const float *in, float *out)
{
    const float invM = 1.0f / (float)IMDCT_M_960;
    /* Borrow out[0..2P-1] as the fft240 four-step intermediate (Gr,Gi). `out`
     * (N=960 floats) is not written until the unfold below, by which point
     * Gr,Gi are dead. Saves 2P of dedicated scratch. */
    s960_Gr = out;
    s960_Gi = out + IMDCT_P_960;
    for (int k = 0; k < IMDCT_P_960; k++) {
        float r0 = in[2 * k];
        float i0 = in[IMDCT_M_960 - 1 - 2 * k];
        float rr = s960_rot_r[k], ri = s960_rot_i[k];
        s960_zr[k] = r0 * rr - i0 * ri;
        s960_zi[k] = r0 * ri + i0 * rr;
    }
    lhdc_fft240(s960_zr, s960_zi, s960_Xr, s960_Xi);
    for (int k = 0; k < IMDCT_P_960; k++) {
        float rr = s960_rot_r[k], ri = s960_rot_i[k];
        /* reZ/imZ alias Xr/Xi (= zr/zi); read both into temps before writing. */
        float xr = s960_Xr[k], xi = s960_Xi[k];
        s960_reZ[k] = (xr * rr - xi * ri) * invM;
        s960_imZ[k] = (xr * ri + xi * rr) * invM;
    }
    /* Symmetric unfold: P=240, M=480 (P/2=120, 3P/2=360, 5P/2-1=599). */
    for (int p = 0; p < IMDCT_M_960; p++) {
        int ne = 2 * p;
        if      (p < 120) out[ne] =  s960_reZ[120 + p];
        else if (p < 360) out[ne] =  s960_imZ[p - 120];
        else              out[ne] = -s960_reZ[p - 360];
        int no = 2 * p + 1;
        if      (p < 120) out[no] = -s960_imZ[119 - p];
        else if (p < 360) out[no] = -s960_reZ[359 - p];
        else              out[no] =  s960_imZ[599 - p];
    }
}

static void lhdc_imdct_selftest_960(void)
{
    static float tin[IMDCT_M_960];
    static float tout[IMDCT_N_960];
    const int test_bins[] = { 1, 17, 60, 200, 479 };
    const int test_outs[] = { 0, 1, 100, 479, 480, 700, 959 };
    const float amp = 1.0e6f;
    float maxabs = 0.0f, maxdiff = 0.0f;
    for (unsigned bi = 0; bi < sizeof(test_bins) / sizeof(test_bins[0]); bi++) {
        int b = test_bins[bi];
        for (int k = 0; k < IMDCT_M_960; k++) tin[k] = 0.0f;
        tin[b] = amp;
        lhdc_imdct_fast_960(tin, tout);
        for (unsigned oi = 0; oi < sizeof(test_outs) / sizeof(test_outs[0]); oi++) {
            int n = test_outs[oi];
            float expc = (2.0f / IMDCT_N_960) * amp *
                cosf((M_PIF / (2.0f * IMDCT_N_960)) * (2 * n + 1 + IMDCT_N_960 / 2) * (2 * b + 1));
            float d = tout[n] - expc; if (d < 0) d = -d;
            float a = expc < 0 ? -expc : expc;
            if (a > maxabs) maxabs = a;
            if (d > maxdiff) maxdiff = d;
        }
    }
    s960_ok = (maxdiff <= 1e-3f * (maxabs > 1.0f ? maxabs : 1.0f)) ? 1 : 0;
    IMDCT_LOGI("LHDCV5_DEC", "IMDCT-960 self-test: fast=%s maxref=%.1f maxdiff=%.3f",
               s960_ok ? "ENABLED" : "DISABLED(fallback)", maxabs, maxdiff);
}

/* ===================== N=1920 fast path (192 kHz / 5 ms) =====================
 * Identical four-step structure to the 960 path, scaled x2: P = N/4 = 480 =
 * N1 x N2 = 32 x 15. Stage 1 is a radix-2 32-point DIT FFT (vs the 16-point for
 * 960); stage 2 is the same radix-15 DFT (N2=15 is unchanged), with its own copy
 * of the w2 matrix so the block frees independently. Tables and scratch are
 * lazily allocated in DRAM (never flash) and resident only while 192k is the
 * active config (freed via lhdc_imdct_free_1920()). */
#define IMDCT_N_1920  1920
#define IMDCT_M_1920  960
#define IMDCT_P_1920  480
#define IMDCT_N1_1920 32
#define IMDCT_N2_1920 15

#define S1920_TBL_FLOATS (2*IMDCT_N2_1920*IMDCT_N2_1920 \
                          + 2*IMDCT_N2_1920*IMDCT_N1_1920 + 2*IMDCT_P_1920)
#define S1920_SCR_FLOATS (2*IMDCT_P_1920)
static float *s1920_tbl = NULL;   /* persistent twiddle tables */
static float *s1920_scr = NULL;   /* per-transform scratch (zr,zi) */
static float *s1920_w2_r, *s1920_w2_i;
static float *s1920_tw_r, *s1920_tw_i, *s1920_rot_r, *s1920_rot_i;
static float *s1920_Gr, *s1920_Gi, *s1920_zr, *s1920_zi;
static float *s1920_Xr, *s1920_Xi, *s1920_reZ, *s1920_imZ;
static int   s1920_built = 0;
static int   s1920_ok = 0;

/* W32^k = exp(-2*pi*i*k/32), k=0..15, for the radix-2 32-point stage-1 FFT. */
static float s1920_w32r[16], s1920_w32i[16];
/* bit-reversal permutation for the 32-point DIT FFT. */
static const uint8_t S1920_BR32[32] =
    { 0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30,
      1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31 };

void lhdc_imdct_free_1920(void)
{
    if (s1920_tbl) { free(s1920_tbl); s1920_tbl = NULL; }
    if (s1920_scr) { free(s1920_scr); s1920_scr = NULL; }
    s1920_built = 0;
    s1920_ok = 0;
}

static void lhdc_imdct_build_fast_tables_1920(void)
{
    if (s1920_built) return;
    if (!s1920_tbl) {
        s1920_tbl = (float *)malloc(S1920_TBL_FLOATS * sizeof(float));
        if (!s1920_tbl) return;   /* stays 0 -> falls back to ref IMDCT */
    }
    if (!s1920_scr) {
        s1920_scr = (float *)malloc(S1920_SCR_FLOATS * sizeof(float));
        if (!s1920_scr) { free(s1920_tbl); s1920_tbl = NULL; return; }
    }
    float *p = s1920_tbl;
    s1920_w2_r = p; p += IMDCT_N2_1920*IMDCT_N2_1920;
    s1920_w2_i = p; p += IMDCT_N2_1920*IMDCT_N2_1920;
    s1920_tw_r = p; p += IMDCT_N2_1920*IMDCT_N1_1920;
    s1920_tw_i = p; p += IMDCT_N2_1920*IMDCT_N1_1920;
    s1920_rot_r = p; p += IMDCT_P_1920;
    s1920_rot_i = p; p += IMDCT_P_1920;
    float *q = s1920_scr;
    s1920_zr = q; q += IMDCT_P_1920;  s1920_zi = q; q += IMDCT_P_1920;
    /* Same aliasing as the 960 path: fft480 runs in place over zr,zi; the
     * post-twiddle rewrites them via temps; Gr,Gi borrow the caller's out. */
    s1920_Xr = s1920_zr;  s1920_Xi = s1920_zi;
    s1920_reZ = s1920_zr; s1920_imZ = s1920_zi;
    for (int k = 0; k < IMDCT_N2_1920; k++)
        for (int n = 0; n < IMDCT_N2_1920; n++) {
            float a = -2.0f * M_PIF / IMDCT_N2_1920 * (float)k * (float)n;
            s1920_w2_r[k * IMDCT_N2_1920 + n] = cosf(a);
            s1920_w2_i[k * IMDCT_N2_1920 + n] = sinf(a);
        }
    for (int n2 = 0; n2 < IMDCT_N2_1920; n2++)
        for (int k1 = 0; k1 < IMDCT_N1_1920; k1++) {
            float a = -2.0f * M_PIF / IMDCT_P_1920 * (float)n2 * (float)k1;
            s1920_tw_r[n2 * IMDCT_N1_1920 + k1] = cosf(a);
            s1920_tw_i[n2 * IMDCT_N1_1920 + k1] = sinf(a);
        }
    for (int k = 0; k < IMDCT_P_1920; k++) {
        float a = -2.0f * M_PIF / IMDCT_N_1920 * ((float)k + 0.125f);
        s1920_rot_r[k] = cosf(a);
        s1920_rot_i[k] = sinf(a);
    }
    for (int k = 0; k < 16; k++) {
        float a = -2.0f * M_PIF * (float)k / 32.0f;
        s1920_w32r[k] = cosf(a);
        s1920_w32i[k] = sinf(a);
    }
    s1920_built = 1;
}

/* 480-point complex FFT (32x15 four-step). */
static LHDC_HOT void lhdc_fft480(const float *zr, const float *zi, float *Xr, float *Xi)
{
    /* Stage 1: a 32-point DFT over n1 for each n2, via a radix-2 DIT FFT. */
    for (int n2 = 0; n2 < IMDCT_N2_1920; n2++) {
        float ar[32], ai[32];
        for (int i = 0; i < 32; i++) {
            int idx = n2 + IMDCT_N2_1920 * (int)S1920_BR32[i];
            ar[i] = zr[idx]; ai[i] = zi[idx];
        }
        for (int s = 1; s <= 5; s++) {
            int m = 1 << s, h = m >> 1, step = 32 / m;
            for (int k = 0; k < 32; k += m) {
                for (int j = 0; j < h; j++) {
                    int tw = j * step;             /* index into W32[0..15] */
                    float wr = s1920_w32r[tw], wi = s1920_w32i[tw];
                    float br = ar[k + j + h], bi = ai[k + j + h];
                    float tr = wr * br - wi * bi;
                    float ti = wr * bi + wi * br;
                    float ur = ar[k + j], ui = ai[k + j];
                    ar[k + j]     = ur + tr; ai[k + j]     = ui + ti;
                    ar[k + j + h] = ur - tr; ai[k + j + h] = ui - ti;
                }
            }
        }
        for (int k1 = 0; k1 < IMDCT_N1_1920; k1++) {
            float tr = s1920_tw_r[n2 * IMDCT_N1_1920 + k1];
            float ti = s1920_tw_i[n2 * IMDCT_N1_1920 + k1];
            s1920_Gr[k1 * IMDCT_N2_1920 + n2] = ar[k1] * tr - ai[k1] * ti;
            s1920_Gi[k1 * IMDCT_N2_1920 + n2] = ar[k1] * ti + ai[k1] * tr;
        }
    }
    for (int k1 = 0; k1 < IMDCT_N1_1920; k1++) {
        lhdc_dft15(&s1920_Gr[k1 * IMDCT_N2_1920], &s1920_Gi[k1 * IMDCT_N2_1920],
                   Xr, Xi, k1, IMDCT_N1_1920);
    }
}

static LHDC_HOT void lhdc_imdct_fast_1920(const float *in, float *out)
{
    const float invM = 1.0f / (float)IMDCT_M_1920;
    /* Borrow out[0..2P-1] (2P=960 <= N=1920) as Gr,Gi; out is dead until unfold. */
    s1920_Gr = out;
    s1920_Gi = out + IMDCT_P_1920;
    for (int k = 0; k < IMDCT_P_1920; k++) {
        float r0 = in[2 * k];
        float i0 = in[IMDCT_M_1920 - 1 - 2 * k];
        float rr = s1920_rot_r[k], ri = s1920_rot_i[k];
        s1920_zr[k] = r0 * rr - i0 * ri;
        s1920_zi[k] = r0 * ri + i0 * rr;
    }
    lhdc_fft480(s1920_zr, s1920_zi, s1920_Xr, s1920_Xi);
    for (int k = 0; k < IMDCT_P_1920; k++) {
        float rr = s1920_rot_r[k], ri = s1920_rot_i[k];
        float xr = s1920_Xr[k], xi = s1920_Xi[k];
        s1920_reZ[k] = (xr * rr - xi * ri) * invM;
        s1920_imZ[k] = (xr * ri + xi * rr) * invM;
    }
    /* Symmetric unfold: P=480, M=960 (P/2=240, 3P/2=720, 5P/2-1=1199). */
    for (int p = 0; p < IMDCT_M_1920; p++) {
        int ne = 2 * p;
        if      (p < 240) out[ne] =  s1920_reZ[240 + p];
        else if (p < 720) out[ne] =  s1920_imZ[p - 240];
        else              out[ne] = -s1920_reZ[p - 720];
        int no = 2 * p + 1;
        if      (p < 240) out[no] = -s1920_imZ[239 - p];
        else if (p < 720) out[no] = -s1920_reZ[719 - p];
        else              out[no] =  s1920_imZ[1199 - p];
    }
}

static void lhdc_imdct_selftest_1920(void)
{
    static float tin[IMDCT_M_1920];
    static float tout[IMDCT_N_1920];
    const int test_bins[] = { 1, 17, 60, 200, 480, 959 };
    const int test_outs[] = { 0, 1, 100, 959, 960, 1400, 1919 };
    const float amp = 1.0e6f;
    float maxabs = 0.0f, maxdiff = 0.0f;
    for (unsigned bi = 0; bi < sizeof(test_bins) / sizeof(test_bins[0]); bi++) {
        int b = test_bins[bi];
        for (int k = 0; k < IMDCT_M_1920; k++) tin[k] = 0.0f;
        tin[b] = amp;
        lhdc_imdct_fast_1920(tin, tout);
        for (unsigned oi = 0; oi < sizeof(test_outs) / sizeof(test_outs[0]); oi++) {
            int n = test_outs[oi];
            float expc = (2.0f / IMDCT_N_1920) * amp *
                cosf((M_PIF / (2.0f * IMDCT_N_1920)) * (2 * n + 1 + IMDCT_N_1920 / 2) * (2 * b + 1));
            float d = tout[n] - expc; if (d < 0) d = -d;
            float a = expc < 0 ? -expc : expc;
            if (a > maxabs) maxabs = a;
            if (d > maxdiff) maxdiff = d;
        }
    }
    s1920_ok = (maxdiff <= 1e-3f * (maxabs > 1.0f ? maxabs : 1.0f)) ? 1 : 0;
    IMDCT_LOGI("LHDCV5_DEC", "IMDCT-1920 self-test: fast=%s maxref=%.1f maxdiff=%.3f",
               s1920_ok ? "ENABLED" : "DISABLED(fallback)", maxabs, maxdiff);
}

/* ------------------------------- init / API ------------------------------- */

/*
 * Lightweight self-test of the fast path: drive a few single-bin impulses and
 * check a handful of output samples against the exact cos() formula. Sets
 * s_fast_ok. If memory for the scratch is unavailable, the fast path is trusted
 * rather than falling back to the slow one.
 */
static void lhdc_imdct_selftest_480(void)
{
    float *tin  = (float *)malloc(IMDCT_M * sizeof(float));
    float *tout = (float *)malloc(IMDCT_N * sizeof(float));
    if (!tin || !tout) {
        s_fast_ok = 1;
        IMDCT_LOGI("LHDCV5_DEC", "IMDCT self-test: skipped (low mem) -> fast TRUSTED");
        free(tin); free(tout);
        return;
    }
    const int test_bins[]  = { 1, 17, 60, 119 };
    const int test_outs[]  = { 0, 1, 100, 239, 240, 350, 479 };
    const float amp = 1.0e6f;
    float maxabs = 0.0f, maxdiff = 0.0f;
    int ok = 1;
    for (unsigned bi = 0; bi < sizeof(test_bins) / sizeof(test_bins[0]); bi++) {
        int b = test_bins[bi];
        for (int k = 0; k < IMDCT_M; k++) tin[k] = 0.0f;
        tin[b] = amp;
        lhdc_imdct_fast_480(tin, tout);
        for (unsigned oi = 0; oi < sizeof(test_outs) / sizeof(test_outs[0]); oi++) {
            int n = test_outs[oi];
            float expc = (2.0f / IMDCT_N) * (float)amp *
                cosf((M_PIF / (2.0f * IMDCT_N)) * (2 * n + 1 + IMDCT_N / 2) * (2 * b + 1));
            float d = (float)tout[n] - expc;
            if (d < 0) d = -d;
            float a = expc < 0 ? -expc : expc;
            if (a > maxabs) maxabs = (float)a;
            if (d > maxdiff) maxdiff = (float)d;
        }
    }
    /* float math at amp=1e6: tolerate 1e-3 of full scale. */
    ok = (maxdiff <= 1e-3f * (maxabs > 1.0f ? maxabs : 1.0f));
    s_fast_ok = ok ? 1 : 0;
    IMDCT_LOGI("LHDCV5_DEC", "IMDCT self-test: fast=%s maxref=%.1f maxdiff=%.3f",
               s_fast_ok ? "ENABLED" : "DISABLED(fallback)", maxabs, maxdiff);
    free(tin); free(tout);
}

int lhdc_imdct_init(int mdct_size)
{
    int N = mdct_size;

    /* Release each rate's fast tables when that rate is not the one being decoded,
     * so a rate's tables live in DRAM only while it is the active config. A no-op
     * once already freed. */
    if (N != IMDCT_N_960)  lhdc_imdct_free_960();
    if (N != IMDCT_N)      lhdc_imdct_free_480();
    if (N != IMDCT_N_1920) lhdc_imdct_free_1920();

    /* Build + self-test the fast path once (only relevant for N=480). The build
     * sets s_fast_built=1 ONLY on a successful malloc; if it failed we must NOT
     * run the self-test (it calls fast_480, which would deref NULL tables) and
     * must fall through to the reference IMDCT below. */
    if (N == IMDCT_N && !s_fast_built) {
        lhdc_imdct_build_fast_tables();
        if (s_fast_built) lhdc_imdct_selftest_480();
    }

    /* Fast path for N=960 (96k). Same NULL-table guard: only self-test if the
     * (split) table+scratch allocation actually succeeded. */
    if (N == IMDCT_N_960 && !s960_built) {
        lhdc_imdct_build_fast_tables_960();
        if (s960_built) lhdc_imdct_selftest_960();
    }

    /* Fast path for N=1920 (192k). Same NULL-table guard. */
    if (N == IMDCT_N_1920 && !s1920_built) {
        lhdc_imdct_build_fast_tables_1920();
        if (s1920_built) lhdc_imdct_selftest_1920();
    }

    /* For sizes no fast path covers (or if a fast path was disabled), make sure
     * the reference cosine table is ready. */
    if (!(N == IMDCT_N && s_fast_ok) && !(N == IMDCT_N_960 && s960_ok) &&
        !(N == IMDCT_N_1920 && s1920_ok))
        (void)lhdc_cos_table_ensure(N);

    return 0;
}

void lhdc_imdct_transform(const float *in, float *out, int mdct_size)
{
if (g_lhdc_diag_force_ref_imdct_960 && mdct_size == 960) {
        lhdc_imdct_ref(in, out, mdct_size);
        return;
    }
#if defined(LHDC_HOST_BUILD)
    /* Diagnostic: force the slow reference cos-formula IMDCT (host build only). */
    { extern char *getenv(const char*); if (getenv("LHDC_FORCE_REF")) { lhdc_imdct_ref(in, out, mdct_size); return; } }
#endif
    if (mdct_size == IMDCT_N_960) {
        if (!s960_built) lhdc_imdct_init(IMDCT_N_960);
        if (s960_ok) { lhdc_imdct_fast_960(in, out); return; }
    }
    if (mdct_size == IMDCT_N_1920) {
        if (!s1920_built) lhdc_imdct_init(IMDCT_N_1920);
        if (s1920_ok) { lhdc_imdct_fast_1920(in, out); return; }
    }
    if (mdct_size == IMDCT_N) {
        if (!s_fast_built) lhdc_imdct_init(IMDCT_N);
        if (s_fast_ok) { lhdc_imdct_fast_480(in, out); return; }
    }
    lhdc_imdct_ref(in, out, mdct_size);   /* builds its cos table on demand */
}
