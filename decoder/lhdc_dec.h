#ifndef LHDC_DEC_H
#define LHDC_DEC_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LHDC_DEC_VERSION_MAJOR 1
#define LHDC_DEC_VERSION_MINOR 0
#define LHDC_DEC_VERSION_PATCH 0

/* Diagnostic one-shot trace flag (see lhdc_dec.c). Set to 1 to log the first
 * decoded frame's leading-section values; the decoder clears it after logging. */
extern volatile int g_lhdc_trace;

/* Return codes */
typedef enum {
    LHDC_DEC_OK                    =  0,
    LHDC_DEC_ERROR                 = -1,
    LHDC_DEC_INVALID_PARAM         = -2,
    LHDC_DEC_INVALID_HANDLE        = -3,
    LHDC_DEC_NOT_INITIALIZED       = -4,
    LHDC_DEC_BUF_NOT_ENOUGH        = -5,
    LHDC_DEC_BITSTREAM_ERROR       = -6,
    LHDC_DEC_UNSUPPORTED_VERSION   = -7,
    LHDC_DEC_UNSUPPORTED_SR        = -8,
    LHDC_DEC_UNSUPPORTED_FORMAT    = -9,
    LHDC_DEC_NEED_MORE_DATA        = -10,
} lhdc_dec_ret_t;

/* Sample rates */
typedef enum {
    LHDC_DEC_SR_44100  = 44100,
    LHDC_DEC_SR_48000  = 48000,
    LHDC_DEC_SR_96000  = 96000,
    LHDC_DEC_SR_192000 = 192000,
} lhdc_dec_sample_rate_t;

/* Bit depths */
typedef enum {
    LHDC_DEC_BITDEPTH_16 = 16,
    LHDC_DEC_BITDEPTH_24 = 24,
    LHDC_DEC_BITDEPTH_32 = 32,
} lhdc_dec_bitdepth_t;

/* Frame duration */
typedef enum {
    LHDC_DEC_FRAME_5MS  = 5,
    LHDC_DEC_FRAME_7P5MS = 7,
    LHDC_DEC_FRAME_10MS = 10,
} lhdc_dec_frame_duration_t;

/* Version */
typedef enum {
    LHDC_DEC_VERSION_1 = 1,
} lhdc_dec_version_t;

/* Extra functions (from bitstream) */
typedef enum {
    LHDC_DEC_EXT_FUNC_NONE  = 0,
    LHDC_DEC_EXT_FUNC_AR    = (1 << 0),
    LHDC_DEC_EXT_FUNC_JAS   = (1 << 1),
    LHDC_DEC_EXT_FUNC_LARC  = (1 << 2),
    LHDC_DEC_EXT_FUNC_META  = (1 << 3),
} lhdc_dec_ext_func_t;

/* Decoder configuration */
typedef struct {
    lhdc_dec_sample_rate_t   sample_rate;
    lhdc_dec_bitdepth_t      bit_depth;
    lhdc_dec_frame_duration_t frame_duration;
    uint8_t                  channels;
    uint32_t                 max_frame_bytes;
    uint8_t                  lossless_enable;
} lhdc_dec_config_t;

/* Frame information */
typedef struct {
    uint32_t  frame_index;
    uint32_t  encoded_frame_bytes;
    uint32_t  samples_per_channel;
    uint8_t   channels;
    uint32_t  sample_rate;
    uint8_t   bit_depth;
    uint8_t   frame_duration_ms;
    uint8_t   version;
    uint32_t  ext_func_flags;
    uint32_t  target_bitrate;
} lhdc_dec_frame_info_t;

/* Opaque decoder handle */
typedef struct lhdc_decoder_t lhdc_decoder_t;

/*
 * Get the required workspace size for the decoder at a given rate. The work
 * buffers are rate-sized (mdct_size from the band config), so 48k needs far
 * less than 96k. Pass the negotiated sample_rate and frame_duration (ms).
 */
size_t lhdc_dec_get_workspace_size(uint32_t sample_rate, uint8_t frame_duration);

/*
 * Initialize a decoder instance.
 * workspace: pre-allocated buffer of size lhdc_dec_get_workspace_size()
 * config:    decoder configuration (may be NULL for auto-detect from bitstream)
 * Returns a decoder handle, or NULL on failure.
 */
lhdc_decoder_t *lhdc_dec_init(void *workspace, const lhdc_dec_config_t *config);

/*
 * Decode one frame.
 * dec:       decoder handle
 * in_data:   input encoded bitstream data
 * in_bytes:  size of input data in bytes
 * out_pcm:   output PCM buffer (interleaved, native endian)
 * out_samples: max samples (per channel) that out_pcm can hold
 * consumed:  [out] number of bytes consumed from in_data
 * generated: [out] number of samples (per channel) written to out_pcm
 * info:      [out] frame information (may be NULL)
 */
lhdc_dec_ret_t lhdc_dec_decode_frame(
    lhdc_decoder_t *dec,
    const uint8_t  *in_data,
    size_t          in_bytes,
    void           *out_pcm,
    uint32_t        out_samples,
    size_t         *consumed,
    uint32_t       *generated,
    lhdc_dec_frame_info_t *info);

/*
 * Flush internal state (for seeking / discontinuities).
 */
void lhdc_dec_flush(lhdc_decoder_t *dec);

/*
 * Reset decoder to initial state.
 */
void lhdc_dec_reset(lhdc_decoder_t *dec);

/*
 * Get decoder configuration (useful after auto-detect).
 */
lhdc_dec_ret_t lhdc_dec_get_config(lhdc_decoder_t *dec, lhdc_dec_config_t *config);

/*
 * Get string description of a return code.
 */
const char *lhdc_dec_strerror(lhdc_dec_ret_t ret);

#ifdef __cplusplus
}
#endif

#endif /* LHDC_DEC_H */
