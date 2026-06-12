#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "common/bt_trace.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lhdc_dec.h"
#include "lhdc_entropy_dec.h"
#include "lhdc_imdct.h"
#include "osi/allocator.h"
#include "stack/a2dp_vendor_lhdcv5.h"
#include "stack/a2dp_vendor_lhdcv5_decoder.h"

#if (defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE)

/* Bluedroid's LOG_INFO/APPL_TRACE_* are compiled out (CONFIG_BT_STACK_NO_LOG),
 * so use ESP-IDF logging directly for the decode path. */
#define LHDCV5_TAG "LHDCV5_DEC"
#define LHDCV5_LOGI(...) ESP_LOGI(LHDCV5_TAG, __VA_ARGS__)
#define LHDCV5_LOGW(...) ESP_LOGW(LHDCV5_TAG, __VA_ARGS__)
#define LHDCV5_LOGE(...) ESP_LOGE(LHDCV5_TAG, __VA_ARGS__)

/* Per-packet decode logging. Disabled by default: each ESP_LOGI blocks the
 * decode task long enough on the UART to stutter playback. Enable only for
 * bring-up. */
#define LHDCV5_VERBOSE 0
/* Dump a few over-the-air payloads as hex for offline analysis. */
#define LHDCV5_FRAMEDUMP 0
static uint32_t s_dump_sr = 0;
static uint32_t s_fd_count = 0;   /* reset per stream in configure */

/* LHDC frames are small (a 96k/5ms frame is under ~1.3 KB); even a fragmented
 * frame fits in 3 KB. */
#define LHDCV5_FRAGMENT_BUF_SIZE (3 * 1024)
/* Decode output buffer. Must match BT_A2DP_SINK_BUF_LHDCV5 in btc_a2dp_sink.c.
 * Sized so 24-bit content emitted in 32-bit containers fits up to 192k
 * (960*2*4 = 7680 B). */
#define LHDCV5_MAX_PCM_BYTES     (4 * 2000)

/*
 * The decoder workspace is allocated from the main internal heap via
 * heap_caps_malloc(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT) rather than osi_malloc.
 * The bluedroid/osi heap fragments during a call so a large allocation there can
 * fail; the main internal heap keeps a large contiguous region. Allocated once
 * at init, reused across reconfigurations, freed on cleanup.
 */
#define LHDCV5_WS_CAPS (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
typedef struct {
    void *workspace;             /* heap block holding the lhdc_decoder_t + rate-sized tail */
    size_t workspace_size;       /* bytes currently allocated (rate-sized) */
    lhdc_decoder_t *decoder;
    int fragment_size;
    int fragment_count;
    uint8_t fragment[LHDCV5_FRAGMENT_BUF_SIZE];
    decoded_data_callback_t decode_callback;
} tA2DP_LHDCV5_DECODER_CB;

/* Allocated lazily on init, freed on cleanup. The control block embeds the
 * fragment-reassembly buffer; keeping it off .bss frees that RAM whenever LHDC
 * is not the active codec (only one A2DP codec runs at a time). The decoder
 * workspace (cb->workspace) is separately rate-sized in decoder_configure. */
static tA2DP_LHDCV5_DECODER_CB *s_lhdc_cb = NULL;

bool a2dp_lhdcv5_decoder_init(decoded_data_callback_t decode_callback) {
    if (!s_lhdc_cb) {
        s_lhdc_cb = (tA2DP_LHDCV5_DECODER_CB *)calloc(1, sizeof(*s_lhdc_cb));
        if (!s_lhdc_cb) {
            LHDCV5_LOGE("cannot allocate LHDC decoder cb");
            return false;
        }
    }
    tA2DP_LHDCV5_DECODER_CB *cb = s_lhdc_cb;
    void  *ws  = cb->workspace;      /* preserve any already-allocated block across re-init */
    size_t wss = cb->workspace_size;
    memset(cb, 0, sizeof(*cb));
    cb->decode_callback = decode_callback;
    /* Workspace is rate-sized and (re)allocated in decoder_configure once the
     * negotiated sample rate is known. Any existing block is kept here. */
    cb->workspace = ws;
    cb->workspace_size = wss;
    return true;
}

void a2dp_lhdcv5_decoder_cleanup(void) {
    /* Release the lazily-allocated fast-IMDCT tables and entropy FAC models as
     * LHDC is going away, so none of the decoder's scratch lingers in DRAM while
     * another codec is active. */
    lhdc_imdct_free_960();
    lhdc_imdct_free_480();
    lhdc_entropy_free();
    tA2DP_LHDCV5_DECODER_CB *cb = s_lhdc_cb;
    if (!cb) return;
    if (cb->workspace) {
        heap_caps_free(cb->workspace);
    }
    free(cb);
    s_lhdc_cb = NULL;
}

void a2dp_lhdcv5_decoder_configure(const uint8_t* p_codec_info) {
    tA2DP_LHDCV5_DECODER_CB *cb = s_lhdc_cb;
    if (!cb) return;
    tA2DP_LHDCV5_CIE cie;
    if (A2DP_ParseInfoLhdcV5(&cie, p_codec_info, false) != A2D_SUCCESS) {
        LHDCV5_LOGE("failed to parse LHDC V5 CIE");
        return;
    }

    cb->decoder = NULL;   /* re-initialized below */

    lhdc_dec_config_t config = {
        .sample_rate = LHDC_DEC_SR_48000,
        .bit_depth = LHDC_DEC_BITDEPTH_16,
        .frame_duration = LHDC_DEC_FRAME_5MS,   /* LHDC V5 uses 5 ms frames */
        .channels = 2,
        .max_frame_bytes = LHDCV5_FRAGMENT_BUF_SIZE,
        .lossless_enable = 0,
    };

    /* Sample rate from the CIE (44.1 / 48 / 96 / 192 kHz all supported). */
    if (cie.sampleRate & A2DP_LHDCV5_SAMPLING_FREQ_192000) {
        config.sample_rate = LHDC_DEC_SR_192000;
    } else if (cie.sampleRate & A2DP_LHDCV5_SAMPLING_FREQ_96000) {
        config.sample_rate = LHDC_DEC_SR_96000;
    } else if (cie.sampleRate & A2DP_LHDCV5_SAMPLING_FREQ_48000) {
        config.sample_rate = LHDC_DEC_SR_48000;
    } else if (cie.sampleRate & A2DP_LHDCV5_SAMPLING_FREQ_44100) {
        config.sample_rate = LHDC_DEC_SR_44100;
    }

    /* Bit depth: prefer 24-bit if offered, else 16-bit. */
    if (cie.bitsPerSample & A2DP_LHDCV5_BITS_PER_SAMPLE_24) {
        config.bit_depth = LHDC_DEC_BITDEPTH_24;
    } else {
        config.bit_depth = LHDC_DEC_BITDEPTH_16;
    }

    config.channels = 2;

    /* Grow-only workspace, sized for the largest rate (192k, mdct 1920) on the
     * first LHDC config and then reused for every rate with no reallocation.
     *
     * Allocating the maximum size up front, while the heap is still
     * unfragmented, guarantees the 192k workspace fits and keeps the heap stable
     * across LHDC rate switches. A rate-sized grow-only scheme would fail when a
     * mid-stream 48k->192k switch needs to grow the block after streaming has
     * fragmented the heap. The block is freed when the active codec leaves
     * LHDC. */
    {
        size_t need = lhdc_dec_get_workspace_size(LHDC_DEC_SR_192000, config.frame_duration);
        if (!cb->workspace || cb->workspace_size < need) {
            if (cb->workspace) heap_caps_free(cb->workspace);
            cb->workspace = heap_caps_malloc(need, LHDCV5_WS_CAPS);
            if (!cb->workspace) {
                cb->workspace_size = 0;
                LHDCV5_LOGE("configure: cannot allocate %u-byte workspace (largest free %u)",
                            (unsigned)need,
                            (unsigned)heap_caps_get_largest_free_block(LHDCV5_WS_CAPS));
                return;
            }
            cb->workspace_size = need;
            LHDCV5_LOGI("configure: workspace %u bytes (sized for 192k; rate=%u)",
                        (unsigned)need, (unsigned)config.sample_rate);
        }
    }

    cb->decoder = lhdc_dec_init(cb->workspace, &config);
    cb->fragment_size = 0;
    cb->fragment_count = 0;
#if LHDCV5_FRAMEDUMP
    s_dump_sr = config.sample_rate; s_fd_count = 0;   /* reset capture per stream */
#endif

    if (!cb->decoder) {
        LHDCV5_LOGE("lhdc_dec_init failed");
        return;
    }

    LHDCV5_LOGI("decoder configured: sr=%d depth=%d ch=%d dur=%d",
                (int)config.sample_rate, (int)config.bit_depth,
                (int)config.channels, (int)config.frame_duration);
    g_lhdc_trace = 0;   /* verbose per-frame decode trace off */
}

ssize_t a2dp_lhdcv5_decoder_decode_packet_header(BT_HDR* p_buf) {
    if (!p_buf) return -EINVAL;

    const size_t header_len = sizeof(struct media_packet_header) +
                              sizeof(struct media_payload_header);
    if (p_buf->len < header_len) {
        APPL_TRACE_ERROR("%s: packet too short", __func__);
        return -EINVAL;
    }

    tA2DP_LHDCV5_DECODER_CB *cb = s_lhdc_cb;
    if (!cb) return -EINVAL;
    uint8_t* src = ((uint8_t *)(p_buf + 1)) + p_buf->offset;
    struct media_payload_header* payload =
        (struct media_payload_header*)(src + sizeof(struct media_packet_header));

#if LHDCV5_VERBOSE
    /* Periodic confirmation that A2DP media packets reach the LHDC decoder. */
    static uint32_t s_hdrs = 0;
    if ((s_hdrs++ % 200) == 0) {
        LHDCV5_LOGI("pkt hdr#%u: len=%u frag=%d first=%d last=%d frame_count=%u",
                    s_hdrs, (unsigned)p_buf->len, payload->is_fragmented,
                    payload->is_first_fragment, payload->is_last_fragment,
                    payload->frame_count);
    }
#endif


    if (payload->is_fragmented) {
        if (payload->is_first_fragment) {
            cb->fragment_size = 0;
        } else if (payload->frame_count + 1 != cb->fragment_count ||
                   (payload->frame_count == 1 && !payload->is_last_fragment)) {
            cb->fragment_count = 0;
            cb->fragment_size = 0;
            return -EINVAL;
        }
        cb->fragment_count = payload->frame_count;
    } else {
        /* Non-fragmented packet: decode_packet parses the self-delimiting LHDC
         * frame stream directly (looping on each frame's [u16 len] header and
         * not using frame_count), so any count is acceptable here, including 0.
         * The LHDC payload's leading byte is a seq/marker byte rather than a
         * frame counter, so this field is not meaningful. If a payload has no
         * decodable frame, decode_packet returns false (frames==0) and the
         * packet is dropped there. */
        cb->fragment_count = 0;
        cb->fragment_size = 0;
    }

    p_buf->offset += header_len;
    p_buf->len -= header_len;
    return 0;
}

bool a2dp_lhdcv5_decoder_decode_packet(BT_HDR* p_buf, unsigned char* buf,
                                       size_t buf_len) {
    tA2DP_LHDCV5_DECODER_CB *cb = s_lhdc_cb;
    if (!cb) return false;
#if LHDCV5_VERBOSE
    {
        static uint32_t s_enter = 0;
        if ((s_enter++ % 200) == 0) {
            LHDCV5_LOGI("decode_packet entered #%u: p_buf=%p buf=%p dec=%p cb=%p buf_len=%u",
                        s_enter, (void*)p_buf, (void*)buf, (void*)cb->decoder,
                        (void*)cb->decode_callback, (unsigned)buf_len);
        }
    }
#endif
    if (!p_buf || !buf || !cb->decoder || !cb->decode_callback) {
        static uint32_t s_e = 0;
        if ((s_e++ % 200) == 0) LHDCV5_LOGW("decode_packet null guard (dec=%p)", (void*)cb->decoder);
        return false;
    }
    if (buf_len < LHDCV5_MAX_PCM_BYTES) {
        static uint32_t s_b = 0;
        if ((s_b++ % 200) == 0) LHDCV5_LOGW("decode buf too small: %u < %u",
                                            (unsigned)buf_len, (unsigned)LHDCV5_MAX_PCM_BYTES);
        return false;
    }

    uint8_t* src = ((uint8_t *)(p_buf + 1)) + p_buf->offset;
    size_t src_size = p_buf->len;

#if LHDCV5_FRAMEDUMP
    /* Dump a burst of over-the-air payloads as hex for offline analysis, tagged
     * FDUMP. Captures one consecutive run per stream, past startup. */
    {
        uint32_t i = s_fd_count++;
        if (i >= 60 && i < 140) {   /* one consecutive burst (80 frames) */
            char line[1600]; int p = 0;
            p += snprintf(line+p, sizeof(line)-p, "FDUMP#%u len=%u sr=%u:", i,
                          (unsigned)src_size, (unsigned)s_dump_sr);
            for (size_t b = 0; b < src_size && p < (int)sizeof(line)-4; b++)
                p += snprintf(line+p, sizeof(line)-p, "%02x", src[b]);
            LHDCV5_LOGI("%s", line);
        }
    }
#endif

    if (cb->fragment_count > 0) {
        if (src_size > (sizeof(cb->fragment) - (size_t)cb->fragment_size)) {
            cb->fragment_count = 0;
            cb->fragment_size = 0;
            APPL_TRACE_ERROR("%s: fragmented LHDC frame too large", __func__);
            return false;
        }

        memcpy(cb->fragment + cb->fragment_size, src, src_size);
        cb->fragment_size += (int)src_size;

        if (cb->fragment_count > 1) return true;

        src = cb->fragment;
        src_size = (size_t)cb->fragment_size;
        cb->fragment_count = 0;
        cb->fragment_size = 0;
    }

    /*
     * One A2DP payload is a concatenation of LHDC V5 frames, each
     * [u16 LE length][payload]. Decode each frame in a loop until the input is
     * exhausted, emitting its PCM to the sink callback immediately rather than
     * accumulating (DRAM is scarce). `buf` is the caller-provided PCM scratch,
     * large enough for one frame (1920 samples).
     *
     * The payload begins with a single leading byte (an LHDC seq/marker byte)
     * before the frame stream; the frames themselves start at offset 1. Skip
     * that byte.
     */
    size_t off = (src_size >= 1) ? 1 : 0;
    int frames = 0;
    size_t pcm_emitted = 0;
    uint8_t out_channels = 2, out_depth = 16;
    (void)out_channels; (void)out_depth;

    while (off + 2 <= src_size) {
        size_t consumed = 0;
        uint32_t generated = 0;
        lhdc_dec_frame_info_t info;
        lhdc_dec_ret_t ret = lhdc_dec_decode_frame(cb->decoder, src + off,
                                                   src_size - off, buf, 1920,
                                                   &consumed, &generated, &info);
        if (ret != LHDC_DEC_OK || consumed == 0) {
            static uint32_t s_errs = 0;
            if (frames == 0 && (s_errs++ % 100) == 0) {
                LHDCV5_LOGW("decode ret=%d off=%u/%u (err#%u)",
                            ret, (unsigned)off, (unsigned)src_size, s_errs);
            }
            break;
        }
        out_channels = info.channels;
        out_depth = info.bit_depth;
        size_t pcm_bytes = (size_t)generated * info.channels * (info.bit_depth / 8);
        cb->decode_callback((uint8_t*)buf, pcm_bytes);
        pcm_emitted += pcm_bytes;
        off += consumed;
        frames++;
    }

    if (frames == 0) {
        return false;
    }

#if LHDCV5_VERBOSE
    /* Periodic stats confirming audio is flowing. Disabled by default: the
     * ESP_LOGI blocks the decode task long enough to stutter audio. */
    {
        static uint32_t s_packets = 0, s_frames = 0, s_pcm = 0;
        s_packets++; s_frames += frames; s_pcm += pcm_emitted;
        if ((s_packets % 100) == 1) {
            LHDCV5_LOGI("RX pkt#%u frames=%d pcm=%u (ch=%d depth=%d) totals: %u frames, %u KiB",
                        s_packets, frames, (unsigned)pcm_emitted,
                        out_channels, out_depth, s_frames, s_pcm / 1024);
        }
    }
#endif
    return true;
}

#endif /* defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE) */
