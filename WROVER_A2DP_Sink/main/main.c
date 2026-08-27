/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bt_app_core_utils.h"
#include "bredr_app_common_utils.h"
#include "a2dp_sink_common_utils.h"
#include "a2dp_utils_tags.h"
#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
#include "a2dp_sink_int_codec_utils.h"
#else
#include "a2dp_sink_ext_codec_utils.h"
#endif

/* device name */
static const char local_device_name[] = CONFIG_EXAMPLE_LOCAL_DEVICE_NAME;

/* event for stack up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/********************************
 * STATIC FUNCTION DECLARATIONS
 *******************************/

/* Device callback function */
static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param);

/* GAP callback function */
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/* callback function for A2DP sink */
static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
/* callback function for A2DP sink audio data stream */
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
#else
/* callback function for A2DP sink undecoded audio data */
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf);
#endif

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

static void bt_app_dev_cb(esp_bt_dev_cb_event_t event, esp_bt_dev_cb_param_t *param)
{
    bredr_app_dev_evt_def_hdl(event, param);
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    bredr_app_gap_evt_def_hdl(event, param);
}

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        bt_app_work_dispatch(bt_a2d_evt_def_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL, NULL);
        break;
    }
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_SEP_REG_STATE_EVT: {
#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
        bt_app_work_dispatch(bt_a2d_evt_int_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL, NULL);
#else
        bt_app_work_dispatch(bt_a2d_evt_ext_codec_hdl, event, param, sizeof(esp_a2d_cb_param_t), NULL, NULL);
#endif
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    bt_a2d_data_hdl(data, len);
}
#else
static void bt_app_a2d_audio_data_cb(esp_a2d_conn_hdl_t conn_hdl, esp_a2d_audio_buff_t *audio_buf)
{
    bt_a2d_audio_data_hdl(conn_hdl, audio_buf);
}
#endif

static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    /* when do the stack up, this event comes */
    case BT_APP_EVT_STACK_UP: {
        esp_bt_gap_set_device_name(local_device_name);
        esp_bt_dev_register_callback(bt_app_dev_cb);
        esp_bt_gap_register_callback(bt_app_gap_cb);

        esp_a2d_register_callback(&bt_app_a2d_cb);
        assert(esp_a2d_sink_init() == ESP_OK);

#if CONFIG_EXAMPLE_A2DP_SINK_USE_EXTERNAL_CODEC == FALSE
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
#else
        esp_a2d_mcc_t mcc = {0};
        mcc.type = ESP_A2D_MCT_SBC;
        mcc.cie.sbc_info.samp_freq = ESP_A2D_SBC_CIE_SF_16K |
                                     ESP_A2D_SBC_CIE_SF_32K |
                                     ESP_A2D_SBC_CIE_SF_44K |
                                     ESP_A2D_SBC_CIE_SF_48K;
        mcc.cie.sbc_info.ch_mode = ESP_A2D_SBC_CIE_CH_MODE_MONO |
                                   ESP_A2D_SBC_CIE_CH_MODE_DUAL_CHANNEL |
                                   ESP_A2D_SBC_CIE_CH_MODE_STEREO |
                                   ESP_A2D_SBC_CIE_CH_MODE_JOINT_STEREO;
        mcc.cie.sbc_info.block_len = ESP_A2D_SBC_CIE_BLOCK_LEN_4 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_8 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_12 |
                                     ESP_A2D_SBC_CIE_BLOCK_LEN_16;
        mcc.cie.sbc_info.num_subbands = ESP_A2D_SBC_CIE_NUM_SUBBANDS_4 | ESP_A2D_SBC_CIE_NUM_SUBBANDS_8;
        mcc.cie.sbc_info.alloc_mthd = ESP_A2D_SBC_CIE_ALLOC_MTHD_SNR | ESP_A2D_SBC_CIE_ALLOC_MTHD_LOUDNESS;
        mcc.cie.sbc_info.max_bitpool = 250;
        mcc.cie.sbc_info.min_bitpool = 2;
        /* register stream end point, only support SBC currently */
        esp_a2d_sink_register_stream_endpoint(0, &mcc);
        esp_a2d_sink_register_audio_data_callback(bt_app_a2d_audio_data_cb);
#endif

        /* Get the default value of the delay value */
        esp_a2d_sink_get_delay_value();
        /* Get local device name */
        esp_bt_gap_get_device_name();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/* Periodically report internal-SRAM heap so RAM usage (BT stack + active codec
 * working tables + audio buffers) is visible. Everything runs in internal RAM
 * (PSRAM disabled), and only the active codec's tables are resident, so this is
 * the number that matters. */
#if CONFIG_BT_A2DP_LHDCV5_DECODER
/* Drained here rather than logged from the decoder: an LHDC frame is 5 ms of
 * audio for BOTH channels, so the decode task must never block on the UART (one
 * ~90-char line at 115200 is ~8 ms = two whole frames). The decoder just keeps
 * counters; this core-0, priority-1 task pays the printing cost. */
extern void lhdc_dec_latency_stats_raw(uint32_t *frames, uint32_t *avg_us, uint32_t *max_us,
                                       uint32_t *over, uint32_t *budget_us, uint32_t *worst_bytes,
                                       int reset);
#endif

/* Decoder-task stack margin. Worth watching because it is codec-dependent:
 * libopus is vendored with VAR_ARRAYS, so its scratch is VLAs on this stack and
 * a 48 kHz stereo frame alone needs several KB. free_min == 0 means no audio
 * has been decoded yet. */
extern void btc_a2dp_sink_stack_stats(uint32_t *total, uint32_t *free_min);

static void heap_monitor_task(void *arg)
{
    (void)arg;
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    uint32_t prev_i0 = 0, prev_i1 = 0; int64_t prev_us = 0;
#endif
    for (;;) {
        size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t largest  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        size_t min_ever = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
        /* The BT media-packet allocator draws from MALLOC_CAP_DEFAULT (byte-
         * addressable DRAM). That pool -- NOT total internal -- is what starves at
         * 192k, so report it too: it must stay well above the ~535 B packet size. */
        size_t free_dram = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
        size_t largest_dram = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        ESP_LOGI("HEAP", "internal free: %u B (%u KB) | largest: %u B | min-ever: %u KB || 8-bit DRAM free: %u B | largest: %u B",
                 (unsigned)free_int, (unsigned)(free_int / 1024),
                 (unsigned)largest, (unsigned)(min_ever / 1024),
                 (unsigned)free_dram, (unsigned)largest_dram);
        {
            uint32_t st_total = 0, st_free = 0;
            btc_a2dp_sink_stack_stats(&st_total, &st_free);
            if (st_free)
                ESP_LOGI("DEC-STACK", "A2DP_DECODER stack: %u B total, %u B free at worst (%u%% used)",
                         (unsigned)st_total, (unsigned)st_free,
                         (unsigned)(st_total ? ((st_total - st_free) * 100u) / st_total : 0));
        }
#if CONFIG_BT_A2DP_LHDCV5_DECODER
        /* Real-time margin: max_us is the number that must stay under budget_us
         * (5000 for a normal 5 ms frame, 2500 low-latency). `over` counts frames
         * that missed it -- any nonzero value is an audible risk. */
        {
            uint32_t lf, lavg, lmax, lover, lbud, lbytes;
            lhdc_dec_latency_stats_raw(&lf, &lavg, &lmax, &lover, &lbud, &lbytes, 1);
            if (lf) {
                ESP_LOGI("LHDC-RT", "frames=%u avg=%u us max=%u us budget=%u us (%u%% peak) over=%u worst_frame=%u B",
                         (unsigned)lf, (unsigned)lavg, (unsigned)lmax, (unsigned)lbud,
                         (unsigned)(lbud ? (lmax * 100u) / lbud : 0),
                         (unsigned)lover, (unsigned)lbytes);
            }
        }
#endif
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
        /* Per-core CPU load. This decides dual-core feasibility: core 0 runs all the
         * BT + I2S render; one LHDC channel decode is ~40% of a core at 192k, so a
         * parallel-channel decode only helps if core 0 has ~>=40% idle to spare. */
        {
            int64_t now_us = esp_timer_get_time();
            TaskStatus_t st0, st1;
            vTaskGetInfo(xTaskGetIdleTaskHandleForCore(0), &st0, pdFALSE, eInvalid);
            vTaskGetInfo(xTaskGetIdleTaskHandleForCore(1), &st1, pdFALSE, eInvalid);
            uint32_t i0 = st0.ulRunTimeCounter;
            uint32_t i1 = st1.ulRunTimeCounter;
            if (prev_us) {
                int64_t win = now_us - prev_us;
                int idle0 = (int)((int64_t)(uint32_t)(i0 - prev_i0) * 100 / win);
                int idle1 = (int)((int64_t)(uint32_t)(i1 - prev_i1) * 100 / win);
                if (idle0 > 100) idle0 = 100;
                if (idle1 > 100) idle1 = 100;
                ESP_LOGI("CPU", "core0 busy=%d%% (idle %d%%)  |  core1 busy=%d%% (idle %d%%)",
                         100 - idle0, idle0, 100 - idle1, idle1);
            }
            prev_i0 = i0; prev_i1 = i1; prev_us = now_us;
        }
#endif
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/*******************************
 * MAIN ENTRY POINT
 ******************************/

void app_main(void)
{
    ESP_ERROR_CHECK(bredr_app_common_init());

    bt_app_task_start_up();
    /* bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL, NULL);

    /* start the internal-RAM heap monitor (prints every 5 s). Pin it to core 0 so
     * it never lands on core 1 and steals cycles / blocks the decode task on UART --
     * core 1 is dedicated to the real-time LHDC decoder. */
    xTaskCreatePinnedToCore(heap_monitor_task, "heap_mon", 3072, NULL, 1, NULL, 0);
}
