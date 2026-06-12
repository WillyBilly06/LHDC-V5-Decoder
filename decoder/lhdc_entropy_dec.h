#   ifndef LHDC_ENTROPY_DEC_H
#define LHDC_ENTROPY_DEC_H

#include "lhdc_dec_internal.h"

/*
 * Full entropy decode (FAC-MA range coder + Rice). Scratch buffers are provided
 * by the caller from the decoder workspace to avoid large .bss.
 *   count   = number of significant coefficients (nzc)
 *   split   = Rice split (0 -> pivot=count)
 *   ma_win1 = MA sliding window for stream 1; ma_win2 = stream 2 (= win1*8)
 *   fac_bytes_out = bytes the range coder consumed (sign-plane offset)
 *   scratch_fac[scratch_fac_cap], scratch_s1[>=count], scratch_s2[scratch_s2_cap]
 */
lhdc_dec_ret_t lhdc_entropy_decode_spectrum_ex2(int32_t *quant_spectrum,
                                                 int num_coeffs,
                                                 lhdc_dec_bit_reader_t *br,
                                                 int count, int split,
                                                 int ma_win1, int ma_win2,
                                                 int *fac_bytes_out,
                                                 uint8_t *scratch_fac, int scratch_fac_cap,
                                                 uint8_t *scratch_s1,
                                                 uint8_t *scratch_s2, int scratch_s2_cap);

/*
 * Allocate the FAC adaptive models. Intended to be called at decoder configure
 * time, while the heap is still contiguous. (In this build the models are static
 * storage, so these are no-ops; the entry points are retained for the API.)
 */
void lhdc_entropy_alloc(void);

/*
 * Release the FAC adaptive models. Safe to call when nothing is allocated.
 */
void lhdc_entropy_free(void);

/*
 * Decode band prediction coding (BPC) parameters and apply inverse BPC.
 */
lhdc_dec_ret_t lhdc_entropy_decode_bpc(int32_t *quant_spectrum,
                                         int num_coeffs,
                                         lhdc_dec_bit_reader_t *br);

#endif /* LHDC_ENTROPY_DEC_H */
