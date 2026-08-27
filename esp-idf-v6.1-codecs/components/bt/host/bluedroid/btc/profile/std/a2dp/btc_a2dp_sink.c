/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/******************************************************************************
 **
 **  Name:          btc_a2dp_sink.c
 **
 ******************************************************************************/
#include "common/bt_target.h"
#include "common/bt_trace.h"
#include <string.h>
#include <stdint.h>
#include "common/bt_defs.h"
#include "osi/allocator.h"
#include "osi/mutex.h"
#include "osi/thread.h"
#include "osi/fixed_queue.h"
#include "stack/a2d_api.h"
#include "bta/bta_av_api.h"
#include "bta/bta_av_ci.h"
#include "btc_av_co.h"
#include "btc_a2dp.h"
#include "btc_a2dp_control.h"
#include "btc_a2dp_sink.h"
#include "btc/btc_manage.h"
#include "btc_av.h"
#include "btc/btc_util.h"
#include "stack/a2dp_codec_api.h"
#include "esp_a2dp_api.h"
#include "oi_status.h"
#include "osi/future.h"
#include <assert.h>
#include "esp_heap_caps.h"  // For heap_caps_get_free_size/largest_free_block

#if (BTC_AV_SINK_INCLUDED == TRUE)

/* Decode scratch is allocated per selected codec (one shared buffer, sized for
 * the active codec, freed/realloc'd on codec switch — never per-codec coexisting). */
#define BT_A2DP_SINK_BUF_SBC        (1024)
/* AAC one-frame PCM scratch. AAC-LC emits up to 1024 samples/ch; stereo S16 =
 * 1024*2*2 = 4096 B (SBR doubling -> 8192). 8 KB covers it. Previously AAC kept
 * its OWN static 8 KB output_buffer in .bss (always resident even when another
 * codec was active); it now decodes into this shared buffer instead. */
#define BT_A2DP_SINK_BUF_AAC        (4 * 1024)
/* LDAC one-frame PCM scratch. The decoder flushes per frame, so this only
 * needs to hold a single frame's interleaved PCM. The largest case is the
 * 2x rates (88.2/96 kHz) which use 256 samples/ch; stereo S24 = 2*3*256 =
 * 1536 bytes, S32/F32 = 2*4*256 = 2048. Size for the worst case. A 1024-byte
 * buffer (sized for 128-sample 44.1/48 kHz frames) overflowed at 96 kHz and
 * corrupted the adjacent LDAC decode structs. */
#define BT_A2DP_SINK_BUF_LDAC       (2048)
#define BT_A2DP_SINK_BUF_APTX       (4*1024)
#define BT_A2DP_SINK_BUF_OPUS       (4*2000)
#define BT_A2DP_SINK_BUF_LC3PLUS    (4*1024)
/* LHDC V5 decode output buffer (one frame's PCM). Must match LHDCV5_MAX_PCM_BYTES
 * in a2dp_vendor_lhdcv5_decoder.c. */
/* 4 KB: a 96k/24b frame decodes to 480*2 samples and 192k to 960*2; emitting in
 * 32-bit containers (4 B/sample) the worst case (192k) is 960*2*4 = 7680 B, so
 * 6 KB would overflow at 192k. 8 KB covers it. Must match LHDCV5_MAX_PCM_BYTES. */
#define BT_A2DP_SINK_BUF_LHDCV5     (6*2600)
#define BT_A2DP_SINK_BUF_DEFAULT    (4*1024)

/*****************************************************************************
 **  Constants
 *****************************************************************************/

/* BTC media cmd event definition : BTC_MEDIA_TASK_CMD */
enum {
    BTC_MEDIA_TASK_SINK_INIT,
    BTC_MEDIA_TASK_SINK_CLEAN_UP,
    BTC_MEDIA_FLUSH_AA_RX,
    BTC_MEDIA_AUDIO_SINK_CFG_UPDATE,
    BTC_MEDIA_AUDIO_SINK_CLEAR_TRACK,
};

enum {
    BTC_A2DP_SINK_STATE_OFF = 0,
    BTC_A2DP_SINK_STATE_ON = 1,
    BTC_A2DP_SINK_STATE_SHUTTING_DOWN = 2
};

/*
 * CONGESTION COMPENSATION CTRL ::
 *
 * Thus setting controls how many buffers we will hold in media task
 * during temp link congestion. Together with the stack buffer queues
 * it controls much temporary a2dp link congestion we can
 * compensate for. It however also depends on the default run level of sinks
 * jitterbuffers. Depending on type of sink this would vary.
 * Ideally the (SRC) max tx buffer capacity should equal the sinks
 * jitterbuffer runlevel including any intermediate buffers on the way
 * towards the sinks codec.
 */

/* fixme -- define this in pcm time instead of buffer count */

/* The typical runlevel of the tx queue size is ~1 buffer
   but due to link flow control or thread preemption in lower
   layers we might need to temporarily buffer up data */

/* Queue depth: encoded-packet RX buffer (the "Bluetooth prebuffer"). 16 entries
 * (~700 B/pkt -> ~11 KB peak) gives ~80 ms of link-jitter buffering, plenty on
 * top of the decoded PCM jitter ring. Reduced 24->16 to free ~5-6 KB of peak
 * heap (the PCM ring already covers decode-scheduling jitter); back-pressure
 * (enqueueFromBt) flow-controls the source a touch earlier, which is fine. */
#define MAX_OUTPUT_A2DP_SNK_FRAME_QUEUE_SZ     (16)

#define BTC_A2DP_SNK_DATA_QUEUE_IDX            (1)

#define A2DP_TASK_NAME                   "A2DP_DECODER"
/* Sized for the HUNGRIEST decoder, which is Opus by a wide margin.
 *
 * The claim this used to carry -- "decoders use pre-allocated static context,
 * not deep stack recursion" -- is false for libopus. It is vendored here built
 * with VAR_ARRAYS (see components/bt/CMakeLists.txt), so every CELT scratch
 * buffer is a C99 VLA on the caller's stack. A 48 kHz stereo frame allocates
 * several KB in one go (celt_sig freq[2*N] alone is 2*960*4 = 7.7 KB in the
 * fixed-point build), which overflowed the old 8 KB stack the instant an Opus
 * stream started:
 *     ***ERROR*** A stack overflow in task A2DP_DECODER has been detected.
 * The PCM output buffer is NOT on this stack (decode_buf is heap), so the whole
 * excess is libopus scratch.
 *
 * 20 KB covers it with headroom. The cost is ~12 KB of internal DRAM, which is
 * affordable here; if it ever isn't, the alternative is rebuilding libopus with
 * NONTHREADSAFE_PSEUDOSTACK so its scratch moves to one static block (safe --
 * only this task decodes). btc_a2dp_sink_stack_stats() reports the measured
 * high-water mark so this can be right-sized from data instead of guessed. */
#define A2DP_TASK_STACK_SIZE             (20480)
#define A2DP_TASK_PRIO                   (BT_TASK_MAX_PRIORITIES - 6)
/* Core 1: this is the task that runs the actual audio decode, and it MUST get a
 * core to itself. Profiled LHDC-v5 worst case (96k / 1000 kbps) is ~92% of one
 * core; the BT controller + host (both pinned to core 0 via CONFIG_BT_CTRL_/
 * BLUEDROID_PINNED_TO_CORE) plus this decode all sharing one core drove that core
 * past 100% -> IDLE watchdog + "codec queue full" drops. Pinning the decode to
 * core 1 (which is otherwise idle) leaves the controller + host + I2S render on
 * core 0 (~65%) and the decode ~alone on core 1 (~92%). The I2S render task is
 * pinned to core 0 (see audio_sink_service_i2s.c) so it never steals core-1 cycles.
 * NOTE: hardcoded here, NOT controlled by CONFIG_BT_BLUEDROID_PINNED_TO_CORE
 * (that pins the BTC signaling task, a different, idle-during-stream task). */
#define A2DP_TASK_PINNED_TO_CORE         (1)
#define A2DP_TASK_WORKQUEUE_NUM          (2)
#define A2DP_TASK_WORKQUEUE0_LEN         (1)
#define A2DP_TASK_WORKQUEUE1_LEN         (4)

typedef struct {
    uint32_t sig;
    void *param;
} a2dp_sink_task_evt_t;

typedef struct {
    BOOLEAN rx_flush; /* discards any incoming data when true */
    struct osi_event *data_ready_event;
    fixed_queue_t *RxSbcQ;
} tBTC_A2DP_SINK_CB;

// typedef struct {
//     uint16_t expected_seq_num;
//     bool seq_num_recount;
// } a2dp_sink_media_pkt_seq_num_t;

typedef struct {
    tBTC_A2DP_SINK_CB   btc_aa_snk_cb;
    osi_thread_t        *btc_aa_snk_task_hdl;
    const tA2DP_DECODER_INTERFACE* decoder;
    unsigned char *decode_buf;  // Allocated from internal RAM
    size_t decode_buf_len;
    btav_a2dp_codec_index_t codec_index;
    BOOLEAN decoder_ready;
    // a2dp_sink_media_pkt_seq_num_t   media_pkt_seq_num;
} a2dp_sink_local_param_t;

static void btc_a2dp_sink_thread_init(UNUSED_ATTR void *context);
static void btc_a2dp_sink_thread_cleanup(UNUSED_ATTR void *context);
static void btc_a2dp_sink_flush_q(fixed_queue_t *p_q);
static void btc_a2dp_sink_rx_flush(void);
/* Handle incoming media packets A2DP SINK streaming*/
/* Minimum free stack observed in A2DP_DECODER (bytes). Starts at "unknown"
 * (UINT32_MAX) so a value only appears once audio has actually been decoded. */
static volatile uint32_t s_dec_stack_free_min = 0xFFFFFFFFu;

/* Struct-free accessor so the app can report it without depending on this
 * component's private headers. total = the configured stack size. */
void btc_a2dp_sink_stack_stats(uint32_t *total, uint32_t *free_min)
{
    if (total)    *total    = A2DP_TASK_STACK_SIZE;
    if (free_min) *free_min = (s_dec_stack_free_min == 0xFFFFFFFFu)
                              ? 0 : s_dec_stack_free_min;
}

static void btc_a2dp_sink_handle_inc_media(BT_HDR *p_msg);
static void btc_a2dp_sink_handle_decoder_reset(tBTC_MEDIA_SINK_CFG_UPDATE *p_msg);
static void btc_a2dp_sink_handle_clear_track(void);
static BOOLEAN btc_a2dp_sink_clear_track(void);

static void btc_a2dp_sink_data_ready(void *context);

/* Free function for queued buffers */
static void btc_a2dp_sink_free_buf(void *buf) {
    if (buf) {
        osi_free(buf);
    }
}

static int btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_OFF;
static esp_a2d_sink_data_cb_t bt_aa_snk_data_cb = NULL;
#if A2D_DYNAMIC_MEMORY == FALSE
static a2dp_sink_local_param_t a2dp_sink_local_param;
#else
static a2dp_sink_local_param_t *a2dp_sink_local_param_ptr;
#define a2dp_sink_local_param (*a2dp_sink_local_param_ptr)
#endif ///A2D_DYNAMIC_MEMORY == FALSE

static size_t btc_a2dp_sink_decode_buf_size(btav_a2dp_codec_index_t codec_index)
{
    switch (codec_index) {
    case BTAV_A2DP_CODEC_INDEX_SINK_SBC:
        return BT_A2DP_SINK_BUF_SBC;
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC:
        return BT_A2DP_SINK_BUF_AAC;
#endif
#if (defined(APTX_DEC_INCLUDED) && APTX_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_APTX:
    case BTAV_A2DP_CODEC_INDEX_SINK_APTX_HD:
    case BTAV_A2DP_CODEC_INDEX_SINK_APTX_LL:
        return BT_A2DP_SINK_BUF_APTX;
#endif
#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LDAC:
        return BT_A2DP_SINK_BUF_LDAC;
#endif
#if (defined(OPUS_DEC_INCLUDED) && OPUS_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_OPUS:
    case BTAV_A2DP_CODEC_INDEX_SINK_OPUS_ANDROID:
        return BT_A2DP_SINK_BUF_OPUS;
#endif
#if (defined(LC3PLUS_DEC_INCLUDED) && LC3PLUS_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LC3PLUS:
        return BT_A2DP_SINK_BUF_LC3PLUS;
#endif
#if (defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LHDCV5:
        return BT_A2DP_SINK_BUF_LHDCV5;
#endif
    default:
        return BT_A2DP_SINK_BUF_DEFAULT;
    }
}

static UINT8 btc_a2dp_sink_queue_limit(void)
{
    switch (a2dp_sink_local_param.codec_index) {
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC:
        return 18;
#endif
#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LDAC:
        return 20;
#endif
#if (defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LHDCV5:
        /* 32: single-core sweet spot (dual-core is disabled -- see lhdc_dec_slot.h).
         * The queue was cut to 12 only to fund the dual-core second scratch set; with
         * that gone the byte-DRAM is free again and the decoder needs the depth to
         * absorb BT jitter + decode spikes. 40+ pushes the 8-bit DRAM pool under its
         * 8192 B floor -> bulk low-memory drops, so 32 is the ceiling. */
        return 32;
#endif
    case BTAV_A2DP_CODEC_INDEX_SINK_SBC:
        return 18;
    default:
        return 18;
    }
}

static UINT8 btc_a2dp_sink_queue_floor(void)
{
    switch (a2dp_sink_local_param.codec_index) {
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC:
        return 8;
#endif
#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LDAC:
        return 8;
#endif
#if (defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LHDCV5:
        return 8;   // try 8 first for 96k/900kbps
#endif
    case BTAV_A2DP_CODEC_INDEX_SINK_SBC:
        return 8;
    default:
        return 8;
    }
}

static size_t btc_a2dp_sink_pressure_target(void)
{
    switch (a2dp_sink_local_param.codec_index) {
#if (defined(AAC_DEC_INCLUDED) && AAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_AAC:
        return 10 * 1024;
#endif
#if (defined(LDAC_DEC_INCLUDED) && LDAC_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LDAC:
        return 6 * 1024;
#endif
#if (defined(LHDCV5_DEC_INCLUDED) && LHDCV5_DEC_INCLUDED == TRUE)
    case BTAV_A2DP_CODEC_INDEX_SINK_LHDCV5:
        return 8 * 1024; 
#endif
    default:
        return 6 * 1024;
    }
}

static BOOLEAN btc_a2dp_sink_configure_decode_buf(size_t required_len)
{
    if (a2dp_sink_local_param.decode_buf_len == required_len &&
        (required_len == 0 || a2dp_sink_local_param.decode_buf != NULL)) {
        return TRUE;
    }

    if (a2dp_sink_local_param.decode_buf) {
        osi_free(a2dp_sink_local_param.decode_buf);
        a2dp_sink_local_param.decode_buf = NULL;
        a2dp_sink_local_param.decode_buf_len = 0;
    }

    if (required_len == 0) {
        return TRUE;
    }

    a2dp_sink_local_param.decode_buf = (unsigned char *)osi_malloc(required_len);
    if (!a2dp_sink_local_param.decode_buf) {
        APPL_TRACE_ERROR("%s: failed to allocate %u byte decode buffer",
                         __func__, (unsigned)required_len);
        return FALSE;
    }
    a2dp_sink_local_param.decode_buf_len = required_len;
    return TRUE;
}

void btc_a2dp_sink_reg_data_cb(esp_a2d_sink_data_cb_t callback)
{
    // todo: critical section protection
    bt_aa_snk_data_cb = callback;
}

static inline void btc_a2d_data_cb_to_app(unsigned char *data, uint32_t len)
{
    // todo: critical section protection
    if (bt_aa_snk_data_cb) {
        bt_aa_snk_data_cb(data, len);
    }
}

/*****************************************************************************
 **  Misc helper functions
 *****************************************************************************/
static inline void btc_a2d_cb_to_app(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    esp_a2d_cb_t btc_aa_cb = (esp_a2d_cb_t)btc_profile_cb_get(BTC_PID_A2DP);
    if (btc_aa_cb) {
        btc_aa_cb(event, param);
    }
}

/*****************************************************************************
 **  BTC ADAPTATION
 *****************************************************************************/

static void btc_a2dp_sink_ctrl(void *param)
{
    BT_HDR* p_buf = (BT_HDR*)param;

    if (!p_buf) {
        APPL_TRACE_ERROR("%s: p_buf is null", __func__);
        return;
    }

    switch (p_buf->event) {
    case BTC_MEDIA_TASK_SINK_INIT:
        btc_a2dp_sink_thread_init(NULL);
        break;
    case BTC_MEDIA_TASK_SINK_CLEAN_UP:
        btc_a2dp_sink_thread_cleanup(NULL);
        break;
    case BTC_MEDIA_AUDIO_SINK_CFG_UPDATE:
        btc_a2dp_sink_handle_decoder_reset(param);
        break;
    case BTC_MEDIA_AUDIO_SINK_CLEAR_TRACK:
        btc_a2dp_sink_handle_clear_track();
        break;
    case BTC_MEDIA_FLUSH_AA_RX:
        btc_a2dp_sink_rx_flush();
        break;
    default:
        APPL_TRACE_WARNING("media task unhandled evt: 0x%x\n", p_buf->event);
    }

    if (param != NULL) {
        osi_free(param);
    }
}

bool btc_a2dp_sink_startup(void)
{
    if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_OFF) {
        APPL_TRACE_ERROR("warning : media task already running");
        return false;
    }

#if A2D_DYNAMIC_MEMORY == TRUE
    if ((a2dp_sink_local_param_ptr = (a2dp_sink_local_param_t *)osi_malloc(sizeof(a2dp_sink_local_param_t))) == NULL) {
        APPL_TRACE_ERROR("%s malloc failed!", __func__);
        return false;
    }
    memset((void *)a2dp_sink_local_param_ptr, 0, sizeof(a2dp_sink_local_param_t));
#endif

    APPL_TRACE_EVENT("## A2DP SINK START MEDIA THREAD ##");

    const size_t workqueue_len[] = {A2DP_TASK_WORKQUEUE0_LEN, A2DP_TASK_WORKQUEUE1_LEN};
    /* Always use internal RAM for task stack - no PSRAM */
    a2dp_sink_local_param.btc_aa_snk_task_hdl = osi_thread_create(
                                    A2DP_TASK_NAME,
                                    A2DP_TASK_STACK_SIZE,
                                    A2DP_TASK_PRIO,
                                    A2DP_TASK_PINNED_TO_CORE,
                                    A2DP_TASK_WORKQUEUE_NUM,
                                    workqueue_len);

    if(!a2dp_sink_local_param.btc_aa_snk_task_hdl) {
        APPL_TRACE_ERROR("%s unable to create a2dp task\n", __func__);
        goto error_exit;
    }

    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        goto error_exit;
    }

    p_buf->event = BTC_MEDIA_TASK_SINK_INIT;
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);

    APPL_TRACE_EVENT("## A2DP SINK MEDIA THREAD STARTED ##\n");

    return true;

error_exit:;
    APPL_TRACE_ERROR("%s unable to start up media thread\n", __func__);

    if (a2dp_sink_local_param.btc_aa_snk_task_hdl) {
        osi_thread_free(a2dp_sink_local_param.btc_aa_snk_task_hdl);
    }
    a2dp_sink_local_param.btc_aa_snk_task_hdl = NULL;

#if A2D_DYNAMIC_MEMORY == TRUE
    osi_free(a2dp_sink_local_param_ptr);
    a2dp_sink_local_param_ptr = NULL;
#endif

    return false;
}

void btc_a2dp_sink_shutdown(void)
{
    APPL_TRACE_EVENT("## A2DP SINK STOP MEDIA THREAD ##\n");

    // Exit thread
    btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_SHUTTING_DOWN;

    /* Post cleanup BEFORE freeing the thread so it actually runs */
    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (p_buf) {
        p_buf->event = BTC_MEDIA_TASK_SINK_CLEAN_UP;
        osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                        btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
    }

    osi_thread_free(a2dp_sink_local_param.btc_aa_snk_task_hdl);

    APPL_TRACE_EVENT("## A2DP SINK MEDIA THREAD STARTED ##\n");

    a2dp_sink_local_param.btc_aa_snk_task_hdl = NULL;

#if A2D_DYNAMIC_MEMORY == TRUE
    osi_free(a2dp_sink_local_param_ptr);
    a2dp_sink_local_param_ptr = NULL;
#endif
}

/*****************************************************************************
**
** Function        btc_a2dp_sink_on_idle
**
*******************************************************************************/

void btc_a2dp_sink_on_idle(void)
{
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    btc_a2dp_sink_rx_flush_req();
    btc_a2dp_sink_clear_track();

    APPL_TRACE_DEBUG("Stopped BT track");
}

/*****************************************************************************
**
** Function        btc_a2dp_sink_on_stopped
**
*******************************************************************************/

void btc_a2dp_sink_on_stopped(tBTA_AV_SUSPEND *p_av)
{
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    btc_a2dp_sink_rx_flush_req();
    btc_a2dp_control_set_datachnl_stat(FALSE);
}

/*****************************************************************************
**
** Function        btc_a2dp_on_suspended
**
*******************************************************************************/

void btc_a2dp_sink_on_suspended(tBTA_AV_SUSPEND *p_av)
{
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    btc_a2dp_sink_rx_flush_req();
    return;
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_clear_track
 **
 ** Description
 **
 ** Returns          TRUE is success
 **
 *******************************************************************************/
static BOOLEAN btc_a2dp_sink_clear_track(void)
{
    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        return false;
    }

    p_buf->event = BTC_MEDIA_AUDIO_SINK_CLEAR_TRACK;
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
    return true;
}

/* when true media task discards any rx frames */
void btc_a2dp_sink_set_rx_flush(BOOLEAN enable)
{
    APPL_TRACE_EVENT("## DROP RX %d ##\n", enable);
    // if (enable == FALSE) {
    //     a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num = 0x1;
    //     a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount = true;
    // }
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = enable;
}

/*****************************************************************************
**
** Function        btc_a2dp_sink_reset_decoder
**
** Description
**
** Returns
**
*******************************************************************************/

void btc_a2dp_sink_reset_decoder(UINT8 *p_av)
{
    APPL_TRACE_EVENT("btc reset decoder");
    APPL_TRACE_DEBUG("btc reset decoder p_codec_info[%x:%x:%x:%x:%x:%x]\n",
                     p_av[1], p_av[2], p_av[3],
                     p_av[4], p_av[5], p_av[6]);

    tBTC_MEDIA_SINK_CFG_UPDATE *p_buf;
    p_buf = osi_malloc(sizeof(tBTC_MEDIA_SINK_CFG_UPDATE));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        return;
    }

    p_buf->hdr.event = BTC_MEDIA_AUDIO_SINK_CFG_UPDATE;
    memcpy(p_buf->codec_info, p_av, AVDT_CODEC_SIZE);
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
}

static void btc_a2dp_sink_data_ready(UNUSED_ATTR void *context)
{
    BT_HDR *p_msg;
    int nb_of_msgs_to_process = 0;

    if (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush == TRUE) {
        btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
        return;
    }

    nb_of_msgs_to_process = fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    APPL_TRACE_DEBUG("nb:%d", nb_of_msgs_to_process);
    while (nb_of_msgs_to_process > 0) {
        if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_ON){
            return;
        }
        p_msg = (BT_HDR *)fixed_queue_dequeue(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, 0);
        if ( p_msg == NULL ) {
            APPL_TRACE_DEBUG("Insufficient data in que ");
            break;
        }

        /* If a flush/reconfig/disconnect happens mid-drain, stop decoding immediately.
         * Decoding stale frames into a reconfigured/closed decoder can corrupt state
         * and has been observed to crash inside the AAC decoder.
         */
        if (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush == TRUE) {
            osi_free(p_msg);
            btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
            return;
        }

        btc_a2dp_sink_handle_inc_media(p_msg);
        /* p_msg is the original BT packet (zero-copy enqueue) */
        osi_free(p_msg);
        nb_of_msgs_to_process--;
    }
    APPL_TRACE_DEBUG(" Process Frames - ");

    if (!fixed_queue_is_empty(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ)) {
        osi_thread_post_event(a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event, OSI_THREAD_MAX_TIMEOUT);
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_handle_decoder_reset
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_handle_decoder_reset(tBTC_MEDIA_SINK_CFG_UPDATE *p_msg)
{
    const tA2DP_DECODER_INTERFACE* decoder = A2DP_GetDecoderInterface(p_msg->codec_info);
    btav_a2dp_codec_index_t codec_index = A2DP_SinkCodecIndex(p_msg->codec_info);
    if (!decoder) {
        APPL_TRACE_ERROR("%s: Couldn't get decoder for codec %s", __func__,
                         A2DP_CodecName(p_msg->codec_info));
        a2dp_sink_local_param.decoder_ready = FALSE;
        a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
        return;
    }

    a2dp_sink_local_param.decoder_ready = FALSE;
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
    if (a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ) {
        btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }

    /* Codec-switch leak accounting. Every decoder here allocates out of the same
     * byte-addressable DRAM pool the BT media packets come from, so a decoder
     * whose cleanup misses a block shows up as free byte-DRAM ratcheting down one
     * switch at a time. Log the pool across this whole function: if in == out the
     * teardown/setup here is balanced and the drift is elsewhere (I2S reconfig,
     * AVDTP), which the SNK_SRV_I2S counterpart of this log will show. */
    size_t dram_in = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    /*
     * Heap-efficiency / fragmentation order (important for large-workspace codecs
     * like LHDC V5, whose rate-sized decoder workspace is ~10-18 KB):
     *
     *   1. Free the OLD decoder's workspace + structs (decoder_cleanup).
     *   2. Free the OLD decode buffer (configure_decode_buf(0)).
     *   3. decoder_init (new) - no big alloc for rate-sized codecs.
     *   4. decoder_configure - allocates the NEW workspace (the LARGEST block)
     *      while the heap is least fragmented. The workspace is rate-sized so it
     *      can only be sized here (after the sample rate is known), NOT in
     *      decoder_init - which is why configure MUST run before the decode buf.
     *   5. Allocate the NEW (small) decode buffer last.
     *
     * Previously the small decode buffer was allocated BEFORE decoder_configure,
     * so it split the heap and left only an ~8.7 KB largest contiguous hole,
     * making the 10.2 KB workspace alloc fail (NULL decoder -> silent packet
     * drop, no audio until enough rate switches defragmented the heap).
     */
    if (decoder != a2dp_sink_local_param.decoder) {
        // De-initialize previous decoder (frees its workspace -> largest free block)
        if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_cleanup) {
            a2dp_sink_local_param.decoder->decoder_cleanup();
        }

        // Free the previous decode buffer too, before the big workspace alloc.
        btc_a2dp_sink_configure_decode_buf(0);

        // Initialize new decoder (rate-sized codecs defer the big alloc to configure)
        a2dp_sink_local_param.decoder = decoder;
        if (a2dp_sink_local_param.decoder->decoder_init &&
            !a2dp_sink_local_param.decoder->decoder_init(btc_a2d_data_cb_to_app)) {
            APPL_TRACE_ERROR("%s: Decoder failed to initialize", __func__);
            a2dp_sink_local_param.decoder = NULL;
            return;
        }
    } else {
        if (a2dp_sink_local_param.decoder->decoder_reset) {
            a2dp_sink_local_param.decoder->decoder_reset();
        }
    }

    // Allocate the BIG (rate-sized) decoder workspace first, while the heap is
    // least fragmented - configure() does this for LHDC V5.
    if (a2dp_sink_local_param.decoder->decoder_configure){
        a2dp_sink_local_param.decoder->decoder_configure(p_msg->codec_info);
    }

    // Allocate the (small) decode buffer LAST, after the big workspace.
    size_t decode_buf_len = btc_a2dp_sink_decode_buf_size(codec_index);
    if (!btc_a2dp_sink_configure_decode_buf(decode_buf_len)) {
        a2dp_sink_local_param.decoder_ready = FALSE;
        a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;
        return;
    }

    a2dp_sink_local_param.codec_index = codec_index;
    a2dp_sink_local_param.decoder_ready = TRUE;
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = FALSE;
    APPL_TRACE_EVENT("%s: codec=%s scratch=%u queue_limit=%u pressure_target=%u",
                     __func__, A2DP_CodecName(p_msg->codec_info),
                     (unsigned)a2dp_sink_local_param.decode_buf_len,
                     (unsigned)btc_a2dp_sink_queue_limit(),
                     (unsigned)btc_a2dp_sink_pressure_target());
    {
        size_t dram_out = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        APPL_TRACE_WARNING("DRAM decoder_reset: in=%u out=%u delta=%+d B (codec=%s)",
                           (unsigned)dram_in, (unsigned)dram_out,
                           (int)((long)dram_out - (long)dram_in),
                           A2DP_CodecName(p_msg->codec_info));
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_handle_inc_media
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_handle_inc_media(BT_HDR *p_msg)
{

    /* XXX: Check if the below check is correct, we are checking for peer to be sink when we are sink */
    if (btc_av_get_peer_sep() == AVDT_TSEP_SNK || (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush)) {
        APPL_TRACE_DEBUG(" State Changed happened in this tick ");
        return;
    }

    if (a2dp_sink_local_param.decoder_ready == FALSE || !a2dp_sink_local_param.decoder) {
        APPL_TRACE_WARNING("%s: dropping media packet before decoder is ready", __func__);
        return;
    }

    if (a2dp_sink_local_param.decode_buf_len != 0 && !a2dp_sink_local_param.decode_buf) {
        APPL_TRACE_WARNING("%s: dropping media packet because decode buffer is unavailable", __func__);
        return;
    }

    // ignore data if no one is listening
    if (!btc_a2dp_control_get_datachnl_stat()) {
        return;
    }

    // if (p_msg->layer_specific != a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num) {
    //     /* Because the sequence number of some devices is not recounted */
    //     if (!a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount ||
    //             a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num != 0x1) {
    //         APPL_TRACE_WARNING("Sequence numbers error, recv:0x%x, expect:0x%x, recount:0x%x",
    //                             p_msg->layer_specific, a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num,
    //                             a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount);
    //     }
    // }
    // a2dp_sink_local_param.media_pkt_seq_num.expected_seq_num  = p_msg->layer_specific + 1;
    // a2dp_sink_local_param.media_pkt_seq_num.seq_num_recount = false;

    if (a2dp_sink_local_param.decoder->decode_packet_header) {
        ssize_t res = a2dp_sink_local_param.decoder->decode_packet_header(p_msg);
        if (res < 0) {
            return;
        }
    }

    if (a2dp_sink_local_param.decoder->decode_packet) {
        unsigned char* buf = a2dp_sink_local_param.decode_buf;
        size_t buf_len = a2dp_sink_local_param.decode_buf_len;
        a2dp_sink_local_param.decoder->decode_packet(p_msg, buf, buf_len);
    }

    /* Smallest free stack seen in this task, in bytes. Sampled here because
     * this is the deepest point of the decode: libopus allocates its scratch as
     * VLAs on this stack, so the margin is codec-dependent and worth watching
     * rather than assuming. Reported by btc_a2dp_sink_stack_stats(). */
    {
        UBaseType_t words = uxTaskGetStackHighWaterMark(NULL);
        uint32_t bytes = (uint32_t)words * sizeof(StackType_t);
        if (bytes < s_dec_stack_free_min) s_dec_stack_free_min = bytes;
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_rx_flush_req
 **
 ** Description
 **
 ** Returns          TRUE is success
 **
 *******************************************************************************/
BOOLEAN btc_a2dp_sink_rx_flush_req(void)
{
    if (fixed_queue_is_empty(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ) == TRUE) { /*  Que is already empty */
        return TRUE;
    }

    BT_HDR *p_buf = (BT_HDR*)osi_malloc(sizeof(BT_HDR));
    if (!p_buf) {
        APPL_TRACE_ERROR("%s: no memory", __func__);
        return false;
    }

    p_buf->event = BTC_MEDIA_FLUSH_AA_RX;
    osi_thread_post(a2dp_sink_local_param.btc_aa_snk_task_hdl,
                    btc_a2dp_sink_ctrl, p_buf, 0, OSI_THREAD_MAX_TIMEOUT);
    return true;
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_rx_flush
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_rx_flush(void)
{
    /* Flush all enqueued SBC  buffers (encoded) */
    APPL_TRACE_DEBUG("btc_a2dp_sink_rx_flush");

    btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_enque_buf
 **
 ** Description      This function is called by the av_co to fill A2DP Sink Queue
 **
 **
 ** Returns          size of the queue
 *******************************************************************************/

UINT8 btc_a2dp_sink_enque_buf(BT_HDR *p_pkt)
{
    if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_ON){
        osi_free(p_pkt);  /* Free original - caller expects us to take ownership */
        return 0;
    }

    if (!a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ ||
        !a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event) {
        osi_free(p_pkt);
        return 0;
    }

    if (a2dp_sink_local_param.btc_aa_snk_cb.rx_flush == TRUE ||
        a2dp_sink_local_param.decoder_ready == FALSE) { /* Flush enabled, do not enque*/
        osi_free(p_pkt);  /* Free original - caller expects us to take ownership */
        return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }

    UINT8 queue_limit = btc_a2dp_sink_queue_limit();
    if (fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ) >= queue_limit) {
        /* Rate-limit this warning HARD. At 192k the UART (115200) takes ~3-4 ms per
         * line; logging every drop floods the UART, and the decode task blocks on
         * the shared log lock -> it falls further behind -> more drops. Logging the
         * 1st then every 512th drop breaks that spiral (heartbeat only). */
        static uint32_t s_qfull_drops = 0;
        if ((s_qfull_drops++ & 0x1FF) == 0) {
            APPL_TRACE_WARNING("Pkt dropped, codec queue full (%u), total=%u\n",
                               (unsigned)queue_limit, (unsigned)s_qfull_drops);
        }
        osi_free(p_pkt);  /* Free original - caller expects us to take ownership */
        return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }

    /* Proactive memory pressure check — only every 32 packets  to avoid
     * the overhead of heap_caps_get_free_size (spinlock + metadata walk)
     * on every incoming BT packet.  At typical A2DP rates (~50 pkt/s)
     * this still checks roughly twice per second. */
    {
        static uint8_t enq_counter = 0;
        if ((++enq_counter & 0x1F) == 0) {
            size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            size_t pressure_target = btc_a2dp_sink_pressure_target();
            if (free_internal < pressure_target) {
                int queue_len = fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
                int queue_floor = btc_a2dp_sink_queue_floor();
                if (queue_len > queue_floor) {
                    int to_drop = queue_len - queue_floor;
                    APPL_TRACE_WARNING("Low memory (%u bytes free, target %u), dropping %d oldest packets",
                                     (unsigned)free_internal, (unsigned)pressure_target, to_drop);
                    for (int i = 0; i < to_drop; i++) {
                        void *buf = fixed_queue_dequeue(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, 0);
                        if (buf) {
                            osi_free(buf);
                        }
                    }
                }
            }
        }
    }

    APPL_TRACE_DEBUG("btc_a2dp_sink_enque_buf + ");

    /* Zero-copy enqueue: pass the original packet directly to the queue
     * instead of malloc+memcpy+free. The packet was allocated with osi_malloc
     * by the HCI/L2CAP layer, so osi_free works on the decode side.
     * This eliminates one malloc+free per packet and avoids transient
     * double-allocation spikes that cause OOM during LDAC streaming. */
    if (!fixed_queue_enqueue(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, p_pkt, 0)) {
        osi_free(p_pkt);
        return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }
    osi_thread_post_event(a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event, 0);
    return fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
}

static void btc_a2dp_sink_handle_clear_track (void)
{
    APPL_TRACE_DEBUG("%s", __FUNCTION__);

    /* Do not clean up the decoder here.
     * Some phones briefly disconnect/reconnect while changing codecs and can
     * deliver the new codec config before the idle/clear-track event is handled.
     * Clearing the decoder here leaves the subsequent STARTED stream muted until
     * a full reconnect. The decoder is safely replaced in decoder_reset() and
     * freed during media-thread cleanup.
     */
    if (a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ) {
        btc_a2dp_sink_flush_q(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_flush_q
 **
 ** Description
 **
 ** Returns          void
 **
 *******************************************************************************/
static void btc_a2dp_sink_flush_q(fixed_queue_t *p_q)
{
    while (! fixed_queue_is_empty(p_q)) {
        void *buf = fixed_queue_dequeue(p_q, 0);
        if (buf) {
            osi_free(buf);
        }
    }
}

static void btc_a2dp_sink_thread_init(UNUSED_ATTR void *context)
{
    APPL_TRACE_EVENT("%s\n", __func__);
    memset(&a2dp_sink_local_param.btc_aa_snk_cb, 0, sizeof(a2dp_sink_local_param.btc_aa_snk_cb));
    a2dp_sink_local_param.decoder_ready = FALSE;
    a2dp_sink_local_param.decode_buf = NULL;
    a2dp_sink_local_param.decode_buf_len = 0;
    a2dp_sink_local_param.codec_index = BTAV_A2DP_CODEC_INDEX_MAX;

    btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_ON;

    struct osi_event *data_event = osi_event_create(btc_a2dp_sink_data_ready, NULL);
    assert (data_event != NULL);
    osi_event_bind(data_event, a2dp_sink_local_param.btc_aa_snk_task_hdl, BTC_A2DP_SNK_DATA_QUEUE_IDX);
    a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event = data_event;

    a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ = fixed_queue_new(QUEUE_SIZE_MAX);

    btc_a2dp_control_init();
}

static void btc_a2dp_sink_thread_cleanup(UNUSED_ATTR void *context)
{

    if (a2dp_sink_local_param.decoder && a2dp_sink_local_param.decoder->decoder_cleanup) {
        a2dp_sink_local_param.decoder->decoder_cleanup();
        a2dp_sink_local_param.decoder = NULL;
    }
    a2dp_sink_local_param.decoder_ready = FALSE;

    btc_a2dp_control_set_datachnl_stat(FALSE);
    /* Clear task flag */
    btc_a2dp_sink_state = BTC_A2DP_SINK_STATE_OFF;

    btc_a2dp_control_cleanup();

    fixed_queue_free(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, btc_a2dp_sink_free_buf);

    a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ = NULL;

    osi_event_delete(a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event);
    a2dp_sink_local_param.btc_aa_snk_cb.data_ready_event = NULL;

    /* Free decode buffer */
    if (a2dp_sink_local_param.decode_buf) {
        osi_free(a2dp_sink_local_param.decode_buf);
        a2dp_sink_local_param.decode_buf = NULL;
    }
    a2dp_sink_local_param.decode_buf_len = 0;
    a2dp_sink_local_param.codec_index = BTAV_A2DP_CODEC_INDEX_MAX;
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_on_memory_pressure
 **
 ** Description      Called by HCI layer when memory allocation fails.
 **                  Immediately flushes queued audio packets to free internal RAM.
 **                  This is a synchronous flush - directly clears the queue without
 **                  posting a message (which might also fail due to low memory).
 **
 ** Returns          void
 **
 *******************************************************************************/
void btc_a2dp_sink_on_memory_pressure(void)
{
    /* Log memory pressure event with current state */
    size_t free_internal = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    (void)free_internal;
    
    APPL_TRACE_WARNING("%s: Memory pressure! Internal RAM: %u bytes free, sink_state=%d", 
                       __func__, (unsigned)free_internal, btc_a2dp_sink_state);
    
    if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_ON) {
        APPL_TRACE_WARNING("%s: Sink not active, nothing to flush", __func__);
        return;
    }

    /* Enable flush mode to discard new incoming packets */
    a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = TRUE;

    /* Directly flush the queue - don't post a message since we're low on memory */
    if (a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ != NULL) {
        int queue_len = fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
        (void)queue_len;
        APPL_TRACE_WARNING("%s: Queue has %d packets", __func__, queue_len);
        
        int flushed = 0;
        while (!fixed_queue_is_empty(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ)) {
            void *buf = fixed_queue_dequeue(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ, 0);
            if (buf) {
                osi_free(buf);
                flushed++;
            }
        }
        APPL_TRACE_WARNING("%s: Flushed %d packets, internal RAM now: %u bytes", 
                          __func__, flushed, 
                          (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    } else {
        APPL_TRACE_WARNING("%s: RxSbcQ is NULL!", __func__);
    }
    
    /* Re-enable rx only if a decoder is configured. */
    if (a2dp_sink_local_param.decoder_ready == TRUE) {
        a2dp_sink_local_param.btc_aa_snk_cb.rx_flush = FALSE;
    }
}

/*******************************************************************************
 **
 ** Function         btc_a2dp_sink_get_queue_depth
 **
 ** Description      Get the current depth of the RX queue
 **
 ** Returns          Number of packets in queue, or 0 if queue not initialized
 **
 *******************************************************************************/
UINT8 btc_a2dp_sink_get_queue_depth(void)
{
    if (btc_a2dp_sink_state != BTC_A2DP_SINK_STATE_ON ||
        a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ == NULL) {
        return 0;
    }
    return (UINT8)fixed_queue_length(a2dp_sink_local_param.btc_aa_snk_cb.RxSbcQ);
}

#endif /* BTC_AV_SINK_INCLUDED */
