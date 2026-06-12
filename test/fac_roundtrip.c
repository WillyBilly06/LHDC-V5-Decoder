/*
 * fac_roundtrip.c -- encode a KNOWN ternary symbol stream with the library's
 * real FAC-MA coder, dump the bytes, so the Python decoder can be validated
 * bit-exactly against the genuine encoder (isolating the range coder from all
 * frame-framing uncertainty).
 *
 * Uses dlsym'd internals:
 *   void FacEncInit(void* st, uint8* buf, int size);
 *   void FacEncSetMA(void* st, const uint32* freqs, int nsym, int window);
 *   int  FacEncodeSymbolMA(void* st, int sym);
 *   int  FacEncFlush(void* st, uint8* dst);
 * (FAC ctx is ~0x2d10 bytes; allocate generously.)
 *
 * Build: aarch64-linux-android28-clang fac_roundtrip.c -ldl -o fac_roundtrip
 * Run:   ./fac_roundtrip ./liblhdcv5.so
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <dlfcn.h>

typedef void (*fn_init)(void*, uint8_t*, int);
typedef void (*fn_setma)(void*, const uint32_t*, int, int);
typedef int  (*fn_encsym)(void*, int);
typedef int  (*fn_flush)(void*, uint8_t*);

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* unbuffered: keep output before any crash */
    const char *lib = (argc > 1) ? argv[1] : "./liblhdcv5.so";
    void *h = dlopen(lib, RTLD_NOW);
    if (!h) { fprintf(stderr, "dlopen: %s\n", dlerror()); return 1; }

    fn_init   FacEncInit   = (fn_init)dlsym(h, "FacEncInit");
    fn_setma  FacEncSetMA  = (fn_setma)dlsym(h, "FacEncSetMA");
    fn_encsym FacEncSym    = (fn_encsym)dlsym(h, "FacEncodeSymbolMA");
    fn_flush  FacEncFlush  = (fn_flush)dlsym(h, "FacEncFlush");
    if (!FacEncInit || !FacEncSetMA || !FacEncSym || !FacEncFlush) {
        fprintf(stderr, "missing FAC symbol(s): init=%p setma=%p sym=%p flush=%p\n",
                (void*)FacEncInit,(void*)FacEncSetMA,(void*)FacEncSym,(void*)FacEncFlush);
        return 1;
    }

    /* FAC context: be generous (struct ends ~0x2d10 + window hist up to ~2248). */
    uint8_t *ctx = calloc(1, 0x8000);
    uint8_t  buf[4096];
    uint8_t  out[4096];

    uint32_t freq1[3] = {54, 12, 1};   /* fq1 init */
    int window = 281;                  /* MA win1 from ECB dump */

    /* Several controlled streams to pin the coder exactly. */
    int s_long[] = {
        1,0,2,0,1,1,2,2,0,0, 1,2,0,1,0,2,2,1,0,0, 2,2,2,1,0,0,0,1,1,2,
        0,0,0,0,2,1,0,2,0,1, 1,1,0,0,2,0,0,1,0,2
    };
    /* Dump the cumfreq table the library builds, to end all speculation. */
    {
        memset(ctx, 0, 0x8000);
        FacEncInit(ctx, buf, sizeof(buf));
        FacEncSetMA(ctx, freq1, 3, window);
        uint32_t *cf = (uint32_t*)(ctx + 0xb0);   /* cumfreq[] per disasm */
        uint32_t *rawf = (uint32_t*)(ctx + 0xc8);  /* raw freq */
        printf("CUMFREQ(@0xb0)");
        for (int i = 0; i < 6; i++) printf(" %u", cf[i]);
        printf("  RAWF(@0xc8)");
        for (int i = 0; i < 4; i++) printf(" %u", rawf[i]);
        printf("  CTX[0xa0..0xb0]");
        for (int o = 0xa0; o < 0xb0; o += 4) printf(" %u", *(uint32_t*)(ctx+o));
        printf("\n");
    }

    /* repeated single-symbol streams to pin the slice mapping */
    static int r0[40], r1[40], r2[40];
    for (int i = 0; i < 40; i++) { r0[i]=0; r1[i]=1; r2[i]=2; }
    struct { const char *name; int *s; int n; } cases[] = {
        { "one2",   r2, 1 },
        { "two2",   r2, 2 },
        { "three2", r2, 3 },
        { "rep2x40", r2, 40 },
        { "long50", s_long, 50 },
    };

    for (size_t c = 0; c < sizeof(cases)/sizeof(cases[0]); c++) {
        memset(ctx, 0, 0x8000);           /* fresh state per case */
        memset(buf, 0, sizeof(buf));
        memset(out, 0, sizeof(out));
        FacEncInit(ctx, buf, sizeof(buf));
        FacEncSetMA(ctx, freq1, 3, window);
        for (int i = 0; i < cases[c].n; i++) FacEncSym(ctx, cases[c].s[i]);
        /* state right before flush */
        uint32_t lo = *(uint32_t*)(ctx+0);
        uint32_t rg = *(uint32_t*)(ctx+4);
        uint64_t bp = *(uint64_t*)(ctx+0x10);
        uint64_t bb = *(uint64_t*)(ctx+0x08);
        int nbytes = FacEncFlush(ctx, out);
        printf("CASE %s nsym=%d preflush low=%08x range=%08x ptr_adv=%ld nbytes=%d BUF",
               cases[c].name, cases[c].n, lo, rg, (long)(bp-bb), nbytes);
        for (int i = 0; i < nbytes; i++) printf(" %02x", buf[i]);
        printf(" OUT");
        for (int i = 0; i < nbytes; i++) printf(" %02x", out[i]);
        printf("\n");
    }

    /* Read QCOEFF from file (argv[2]) and sweep split through the real Rice+FAC
     * pipeline to find which split reproduces the real frame's FAC bytes
     * (cd b3 49 ...). Prints the matching ternary so we can fix Python rice_dec. */
    typedef int (*fn_rice)(int32_t*, int, int, uint8_t*);
    fn_rice Rice = (fn_rice)dlsym(h, "RiceQuotientEncode");
    const char *qcfile = (argc > 2) ? argv[2] : NULL;
    if (Rice && qcfile) {
        static int32_t qc[512];
        int qn = 0;
        FILE *f = fopen(qcfile, "r");
        if (f) { while (qn < 512 && fscanf(f, "%d", &qc[qn]) == 1) qn++; fclose(f); }
        printf("QN=%d\n", qn);
        static uint8_t tout[8192], fbuf[8192], fout[8192];
        uint32_t fq2[3] = {90, 16, 1};
        int w1 = 281, w2 = 2248;
        /* Sweep split: Rice-encode the coeff file + FAC-encode both streams, and
         * report which split's FAC bytes start cd b3 49 (== the real frame). */
        for (int split = 0; split <= qn; split++) {
            int32_t tmp[600]; memset(tmp,0,sizeof(tmp));
            memcpy(tmp, qc, sizeof(int32_t)*qn);
            memset(tout,0,sizeof(tout));
            int n = Rice(tmp, qn, split, tout);
            if (n < 0 || n > 7000) continue;
            int bad=0; for (int i=0;i<n;i++) if ((tout[i]&0xFF)>2){bad=1;break;}
            if (bad) continue;
            memset(ctx,0,0x8000); memset(fbuf,0,sizeof(fbuf));
            FacEncInit(ctx,fbuf,sizeof(fbuf));
            FacEncSetMA(ctx,freq1,3,w1);
            for (int i=0;i<qn;i++) FacEncSym(ctx,tout[i]&0xFF);
            FacEncSetMA(ctx,fq2,3,w2);
            for (int i=qn;i<n;i++) FacEncSym(ctx,tout[i]&0xFF);
            FacEncFlush(ctx,fout);
            if (fbuf[0]==0xcd&&fbuf[1]==0xb3&&fbuf[2]==0x49)
                printf("COEFFMATCH split=%d n=%d FAC %02x %02x %02x %02x\n",
                       split,n,fbuf[0],fbuf[1],fbuf[2],fbuf[3]);
        }
        printf("COEFFSWEEP-DONE qn=%d\n", qn);
    } else {
        printf("Rice or qcfile missing (argc=%d)\n", argc);
    }

    /* adsq_enc oracle: feed known SF arrays, dump sidebits + reconstructed out. */
    typedef int (*fn_adsq)(int32_t*, int32_t*, int, uint8_t*, int*, int);
    fn_adsq Adsq = (fn_adsq)dlsym(h, "adsq_enc");
    if (Adsq) {
        struct { const char *name; int32_t in[40]; int n; } ac[] = {
            /* my adsq_inverse output for the real frame's sns bits: does encoding
             * it reproduce the frame bits [0,0,0,1,0,1,1,1,1,1,1,0,0,1,1,...]? */
            { "myinv", {0,58,135,250,173,231,186,128,38,-103,-327,-679,-455,-193,-359,-564,-436,-282,-384,-512,-422,-307,-384,-486,-422,-332,-390,-467,-409,-332,-390,-467}, 32 },
            { "realsf",{0,-219,-383,-447,-505,-521,-483,-444,-352,-211,8,146,180,194,166,161,183,189,176,173,179,179,173,173,179,180,176,172,172,172,172,170}, 32 },
        };
        for (size_t k=0;k<sizeof(ac)/sizeof(ac[0]);k++){
            int32_t in[40], out[40]; uint8_t bits[40]; int state=8;
            memcpy(in, ac[k].in, sizeof(int32_t)*ac[k].n);
            memset(out,0,sizeof(out)); memset(bits,0,sizeof(bits));
            Adsq(in, out, ac[k].n, bits, &state, 22);
            printf("ADSQ %s IN", ac[k].name);
            for(int i=0;i<ac[k].n;i++) printf(" %d", ac[k].in[i]);
            printf(" BITS");
            for(int i=0;i<ac[k].n;i++) printf(" %d", bits[i]);
            printf(" OUT");
            for(int i=0;i<ac[k].n;i++) printf(" %d", out[i]);
            printf(" endstate=%d\n", state);
        }
    }

    free(ctx);
    dlclose(h);
    return 0;
}
