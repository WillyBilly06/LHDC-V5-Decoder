/*
 * AAC-LC Decoder using libhelix-aac
 * 
 * Wrapper around the RealNetworks Helix AAC decoder for ESP32.
 * This provides the aac_decoder.h interface using libhelix-aac.
 * 
 * Memory optimization: Uses static pre-allocated buffers to avoid
 * dynamic allocation and reduce memory fragmentation.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "aac_decoder.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

/* Include libhelix-aac headers */
#include "aacdec.h"
#include "aaccommon.h"
#include "coder.h"

#define TAG "AAC_HELIX"

/*******************************************************************************
 * Static Memory Pool - Pre-allocated to avoid malloc
 * 
 * Memory layout:
 *   - AACDecInfo: ~100 bytes
 *   - PSInfoBase: ~26KB (includes coef[2][1024], overlap[2][1024], etc.)
 * 
 * Total: ~27KB static allocation
 ******************************************************************************/

/* Align to 8 bytes for safety */
#define HELIX_BUFFER_SIZE (sizeof(AACDecInfo) + sizeof(PSInfoBase) + 64)

/* ~27 KB working buffer. Allocated lazily on init and freed on deinit so it
 * only occupies DRAM while AAC is the active codec (only one A2DP codec runs
 * at a time). Previously a permanent static .bss array that wasted 20 KB
 * whenever any other codec was in use. */
static uint8_t *s_helix_buffer = NULL;
static HAACDecoder s_helix_dec = NULL;
static AACFrameInfo s_frame_info = {0};
static bool s_initialized = false;

/*******************************************************************************
 * Public API - implements aac_decoder.h interface
 ******************************************************************************/

int aac_decoder_init(aac_decoder_t *dec)
{
    if (!dec) {
        return AAC_ERR_PARAM;
    }
    
    memset(dec, 0, sizeof(aac_decoder_t));
    
    /* Free previous decoder if any */
    if (s_helix_dec) {
        s_helix_dec = NULL;
        s_initialized = false;
    }

    /* Lazily allocate the ~27 KB working buffer (internal RAM, 8-byte aligned
     * for the Helix struct layout). Freed in aac_decoder_deinit. */
    if (!s_helix_buffer) {
        s_helix_buffer = (uint8_t *)heap_caps_aligned_alloc(
            8, HELIX_BUFFER_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!s_helix_buffer) {
            ESP_LOGE(TAG, "Failed to allocate %u-byte AAC buffer", (unsigned)HELIX_BUFFER_SIZE);
            return AAC_ERR_NOMEM;
        }
    }

    /* Clear working buffer */
    memset(s_helix_buffer, 0, HELIX_BUFFER_SIZE);

    /* Initialize using the buffer */
    void *ptr = s_helix_buffer;
    int size = HELIX_BUFFER_SIZE;

    s_helix_dec = AACInitDecoderPre(ptr, size);
    if (!s_helix_dec) {
        ESP_LOGE(TAG, "Failed to init helix AAC decoder");
        heap_caps_free(s_helix_buffer);
        s_helix_buffer = NULL;
        return AAC_ERR_NOMEM;
    }

    memset(&s_frame_info, 0, sizeof(s_frame_info));
    s_initialized = true;

    dec->sample_rate = 44100;
    dec->channels = 2;

    ESP_LOGI(TAG, "Helix AAC decoder initialized (buffer: %u bytes)", (unsigned)HELIX_BUFFER_SIZE);
    return AAC_OK;
}

int aac_decoder_configure(aac_decoder_t *dec, int sample_rate, int channels)
{
    if (!dec) {
        return AAC_ERR_PARAM;
    }
    
    dec->sample_rate = sample_rate;
    dec->channels = channels;
    
    /* Configure helix for raw AAC data (no ADTS headers) */
    AACFrameInfo info = {0};
    info.nChans = channels;
    info.sampRateCore = sample_rate;
    info.profile = AAC_PROFILE_LC;
    
    if (s_helix_dec) {
        AACSetRawBlockParams(s_helix_dec, 0, &info);
        ESP_LOGI(TAG, "Configured: %dHz, %dch (raw AAC mode)", sample_rate, channels);
    }
    
    return AAC_OK;
}

int aac_decoder_decode(aac_decoder_t *dec, const uint8_t *data, int len, 
                       int16_t *pcm_out, int *samples_out)
{
    if (!dec || !data || len < 1 || !pcm_out || !samples_out) {
        return AAC_ERR_PARAM;
    }
    
    if (!s_helix_dec || !s_initialized) {
        ESP_LOGE(TAG, "Decoder not initialized");
        return AAC_ERR_PARAM;
    }
    
    *samples_out = 0;
    
    /* Log occasionally for debugging */
    static uint32_t call_count = 0;
    call_count++;
    if (call_count <= 5 || call_count % 500 == 0) {
        ESP_LOGI(TAG, "Decode #%lu: %d bytes", (unsigned long)call_count, len);
    }
    
    /* Set up input pointers */
    unsigned char *inbuf = (unsigned char *)data;
    int bytes_left = len;
    
    /* Decode AAC frame */
    int err = AACDecode(s_helix_dec, &inbuf, &bytes_left, pcm_out);
    
    if (err == ERR_AAC_NONE) {
        /* Get frame info */
        AACGetLastFrameInfo(s_helix_dec, &s_frame_info);
        
        *samples_out = s_frame_info.outputSamps / s_frame_info.nChans;
        dec->frame_count++;
        
        return AAC_OK;
    } else if (err == ERR_AAC_INDATA_UNDERFLOW) {
        /* Need more data - not a fatal error for streaming */
        return AAC_OK;
    } else {
        /* Decode error */
        dec->error_count++;
        
        if (dec->error_count <= 10 || dec->error_count % 100 == 0) {
            ESP_LOGW(TAG, "Decode error: %d (frame %lu, errs %lu)", 
                     err, 
                     (unsigned long)dec->frame_count,
                     (unsigned long)dec->error_count);
        }
        
        return AAC_ERR_DATA;
    }
}

void aac_decoder_deinit(aac_decoder_t *dec)
{
    s_helix_dec = NULL;
    s_initialized = false;

    /* Release the ~27 KB working buffer back to the heap so it is not resident
     * while another codec is active. */
    if (s_helix_buffer) {
        heap_caps_free(s_helix_buffer);
        s_helix_buffer = NULL;
    }

    memset(&s_frame_info, 0, sizeof(s_frame_info));
    
    if (dec) {
        ESP_LOGI(TAG, "Decoder deinit. Frames: %lu, Errors: %lu",
                 (unsigned long)dec->frame_count,
                 (unsigned long)dec->error_count);
        memset(dec, 0, sizeof(aac_decoder_t));
    }
}

/* Also provide aac_decoder_cleanup as an alias */
void aac_decoder_cleanup(aac_decoder_t *dec)
{
    aac_decoder_deinit(dec);
}

