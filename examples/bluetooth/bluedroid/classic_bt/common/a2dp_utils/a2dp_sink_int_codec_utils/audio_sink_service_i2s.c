/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_a2dp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "driver/i2s_std.h"
#include "audio_sink_service.h"

/* On the classic ESP32 with PSRAM, put the output ring in PSRAM: it frees scarce
 * internal byte-DRAM for the BT pool + decode workspace, and the render task reads
 * it sequentially (NOT in the decode hot loop), so PSRAM latency/cache cost is
 * negligible -- unlike routing BT packets to PSRAM, which cache-thrashed the flash
 * decode code and halved throughput. A PSRAM ring must be created AND deleted with
 * the *WithCaps API. Everywhere else it's a normal internal ring. */
#include "esp_heap_caps.h"
#include "esp_timer.h"
#if defined(CONFIG_IDF_TARGET_ESP32) && defined(CONFIG_SPIRAM)
  #define RING_IN_PSRAM 1
  #define RING_DELETE(rb) vRingbufferDeleteWithCaps(rb)
#else
  #define RING_IN_PSRAM 0
  #define RING_DELETE(rb) vRingbufferDelete(rb)
#endif

/* Byte-DRAM the BT stack still needs AFTER the output ring is created.
 *
 * On the classic ESP32 the ring, the LHDC work buffers, the A2DP task stack and
 * every BT media packet all come out of ONE small pool (MALLOC_CAP_INTERNAL |
 * MALLOC_CAP_8BIT); the rest of SRAM is 32-bit-only IRAM that none of them can
 * use. Everything except the queued packets is already allocated by the time the
 * ring is created (decoder configure and the A2DP task both run first), so a
 * fixed reserve is enough to cover what comes later:
 *   ~22 KB  A2DP RxQ at the LHDC queue limit (32 packets x ~680 B)
 *   ~10 KB  L2CAP/AVDTP/HCI churn (observed failing sizes: 660, 1000, 284, 42)
 *    ~8 KB  headroom
 * Sizing the ring by a compile-time constant instead cannot work across rates:
 * the LHDC 192k work buffers + IMDCT-1920 tables are ~22 KB larger than at 48k,
 * which is exactly the amount that pushed "8-bit DRAM free" to 316 B and turned
 * every BT allocation into "BT_OSI: malloc failed". */
/* MEASURED on device (2026-08-26), not estimated. At 192k the budget line read
 * "8-bit DRAM free 70 KB", the ring took 28 KB, and steady-state streaming free
 * settled at ~39 KB -- so everything allocated AFTER the ring came to about
 * 3 KB, not the ~32 KB this reserve was sized for. The A2DP RxQ never fills to
 * its 32-packet limit because the decoder keeps up on average (69% load), so
 * reserving for a full queue just starved the output ring.
 *
 * 24 KB keeps ~21 KB free while streaming 192k -- two orders of magnitude clear
 * of the 660 B allocation that failed at 316 B free -- and hands the other 16 KB
 * to the ring, where it buys the cushion that stops the underflow oscillation. */
#define RINGBUF_BT_DRAM_RESERVE        (24 * 1024)

/* How long a ring write may WAIT for space before giving up and dropping.
 *
 * This used to be 0 (drop instantly on a full ring) and that was the single
 * biggest source of stutter. Measured over a 13.5 s window at 192 kHz:
 *   20,582,400 B accepted + 245,760 B dropped = 20,828,160 B offered
 *   realtime over the same window            = 20,820,480 B
 * i.e. the source delivered 100.04% of realtime -- essentially exact -- yet we
 * discarded 245 KB during momentary fullness and then underflowed by very close
 * to the amount discarded. Delivery is BURSTY, not fast: the ring level swings,
 * touches the ceiling, we delete audio that is still needed 200 ms later, and
 * the deficit resurfaces as an underflow.
 *
 * Waiting instead applies backpressure. The render task drains 1.5 MB/s at
 * 192 kHz, so 20 ms frees ~30 KB -- far more than any single packet needs. The
 * cost is that the A2DP decode task blocks briefly, which is exactly the right
 * behaviour: the burst then waits in the RxQ in ENCODED form (~250 B per 5 ms
 * frame instead of 7680 B of PCM, ~30x cheaper) rather than being destroyed.
 * If the source really were faster than realtime this timeout would expire and
 * we would drop as before, so the pathological case is still bounded. */
#define RINGBUF_SEND_WAIT_MS           (20)

#if defined(CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_EXTERNAL_I2S)

/* log tag */
#define AUDIO_SNK_SRV_I2S_TAG    "SNK_SRV_I2S"

typedef struct {
    i2s_chan_handle_t tx_chan;        /* handle of i2s channel */
    audio_sink_chan_st_t chan_st;     /* i2s channel status */
    TaskHandle_t write_task_handle;   /* handle of writing task */
    RingbufHandle_t ringbuf;          /* handle of ringbuffer */
    SemaphoreHandle_t write_semaphore;/* handle of write semaphore */
    uint16_t ringbuffer_mode;         /* ringbuffer mode */
    uint8_t  cur_bits;                /* current codec decoder PCM width: 16/24/32 */
    int32_t  cur_calib_q15;           /* per-codec calibration gain, Q15 (32768=unity) */
} audio_sink_srv_i2s_cb_t;

/* Vendor codec IDs (AOSP A2DP) for parsing NON_A2DP raw CIE bytes */
/* NOTE: these duplicate the stack's a2dp_vendor_*_constants.h, which are private
 * to the bt component and not reachable from this example util. Duplication is
 * why the Opus IDs below silently drifted out of sync with what the stack
 * advertises; if you add or change a codec, check both places. */
#define A2DP_LDAC_VENDOR_ID     0x0000012DUL
#define A2DP_LDAC_CODEC_ID      0x00AA
#define A2DP_APTX_VENDOR_ID     0x0000004FUL
#define A2DP_APTX_CODEC_ID      0x0001
#define A2DP_APTXHD_VENDOR_ID   0x000000D7UL
#define A2DP_APTXHD_CODEC_ID    0x0024
#define A2DP_APTXLL_VENDOR_ID   0x0000000AUL
#define A2DP_APTXLL_CODEC_ID    0x0002
/* Two Opus flavours exist on A2DP and BOTH turn up in practice:
 *   - Google/Android IDs, which is what THIS stack advertises (see
 *     a2dp_vendor_opus_constants.h: A2DP_OPUS_ANDROID_*) and what PipeWire
 *     selects with its `opus-g` codec;
 *   - PipeWire/BlueZ "Opus 05".
 * Only the 05 pair used to be listed here, so a Google-Opus stream fell through
 * to the unknown-codec default of 32-bit while the decoder emits 16-bit -- I2S
 * then consumed the ring twice as fast as it filled, giving continuous
 * "ringbuffer underflowed" and stuttering audio. Match both. */
#define A2DP_OPUS_G_VENDOR_ID   0x000000E0UL
#define A2DP_OPUS_G_CODEC_ID    0x0001
#define A2DP_OPUS_05_VENDOR_ID  0x000005F1UL
#define A2DP_OPUS_05_CODEC_ID   0x1005
#define A2DP_LHDCV5_VENDOR_ID   0x0000053AUL
#define A2DP_LHDCV5_CODEC_ID    0x4C35
#define A2DP_LC3PLUS_VENDOR_ID  0x000008A9UL
#define A2DP_LC3PLUS_CODEC_ID   0x0001

/* ---- Unified output-level stage -------------------------------------------
 * Every codec's decoded PCM converges in audio_sink_srv_data_output(), so that
 * is the ONE place to normalize loudness. Two multiplied factors, both Q15:
 *   1. a per-codec CALIBRATION gain (codec_calib_gain_q15) that trims each
 *      decoder to a common reference so switching codec/config never changes
 *      loudness. Ships at unity for every codec -> identical behavior to before
 *      until you MEASURE each codec's analog level and fill the table below.
 *   2. a global user VOLUME (s_volume_q15), 0..unity.
 * Gains are attenuations (<= unity) by design, so the sum never clips; a
 * saturating clamp guards against a mistyped table entry anyway. */
#define A2DP_GAIN_Q15_UNITY   (32768)

static int32_t s_volume_q15 = A2DP_GAIN_Q15_UNITY;   /* global volume, default max */

/* Fraction of the ring the re-prime level may occupy, as a shift-friendly
 * numerator over 4. It used to be 1/2, which at 192k meant a 28 KB ring gave a
 * 14 KB = 9.3 ms cushion -- less time than a single worst-case decoded frame
 * (7.5 ms), so playback re-primed and underflowed again within ~20 ms, over and
 * over. 3/4 leaves a quarter of the ring as burst headroom above the re-prime
 * level while roughly doubling the cushion. */
#define RINGBUF_PREFETCH_NUM   3
#define RINGBUF_PREFETCH_DEN   4

/* Rate-proportional output-ring prefetch/re-prime level, in BYTES. A fixed byte
 * count gives 4x less buffered TIME at 192k than 48k (why only 192k underflowed);
 * this is recomputed per stream from the sample rate to a constant ~42 ms cushion
 * (I2S is always 32-bit stereo = 8 bytes/frame). Default = the compile-time floor
 * until the first codec-config sets it. */
static uint32_t s_prefetch_bytes = RINGBUF_PREFETCH_WATER_LEVEL;

/* Actual capacity of the output ring, in bytes. Normally RINGBUF_HIGHEST_WATER_LEVEL,
 * but a memory-constrained target (a classic ESP32 with PSRAM off has no single
 * ~128 KB contiguous internal block once the BT stack is up — largest is ~108 KB)
 * makes the create below step the request down until it fits. The prefetch re-prime
 * cap tracks THIS, not the macro, so a shrunken ring never sits below its own
 * prefetch level (which would wedge playback in PREFETCHING forever). */
static uint32_t s_ring_capacity = RINGBUF_HIGHEST_WATER_LEVEL;

/* Underflow forensics. When the ring runs dry there are only three possible
 * causes, and they need completely different fixes:
 *
 *   input  < realtime -> the source or the decoder is not producing enough
 *   input == realtime -> delivery is bursty; the cushion is too shallow
 *   input  > realtime -> we are DROPPING on a full ring, then starving after
 *
 * Guessing between them wastes flashes, so measure: count PCM bytes actually
 * accepted into the ring and bytes refused, and at each underflow report what
 * fraction of realtime arrived since the previous one. I2S is always 32-bit
 * stereo, so realtime is exactly sample_rate * 8 bytes/s. */
static uint32_t s_out_rate_hz = 48000;   /* set at codec config */

/* ---- adaptive I2S rate tracking -------------------------------------------
 *
 * The source and this sink run off DIFFERENT crystals, so their sample rates
 * differ by a small constant amount. MEASURED on this board against a phone at
 * 192 kHz: over 220 s of clean playback with zero drops, the input came up
 * ~43 KB short of realtime after accounting for the ring's starting level --
 * about 127 ppm. That drains any finite buffer at a fixed rate: a 44 KB ring
 * (28 ms) empties in roughly 3-4 minutes, which is exactly the observed
 * 'occasionally it stutters, then it's fine again'.
 *
 * No buffer size fixes a rate mismatch; it only sets the countdown. The fix is
 * to make OUR clock follow theirs. i2s_channel_tune_rate() adjusts MCLK (and
 * hence the sample rate) at runtime -- its documented purpose is 'fine-tuning
 * the mclk to match the speed of producer and consumer'.
 *
 * Control law follows the pattern in ESP-IDF's own i2s_usb example: act only on
 * a SUSTAINED trend, never on an instantaneous reading, and step gently.
 * The error signal here is the output ring's fill level, which is a much better
 * observer than the example's DMA water mark: it is ~44 KB deep instead of a few
 * hundred bytes, so it integrates the drift for us.
 *
 * Espressif warn that changing the rate while audio is playing can produce
 * plosive artefacts, so the channel is disabled around the change (costing one
 * DMA buffer, ~4 ms at 192k). That is far cheaper than the ~40 ms gaps a real
 * underflow causes, and once the loop has converged it should stop adjusting. */
#define TUNE_PERIOD_MS      500     /* how often the fill is sampled */
#define TUNE_TREND_SAMPLES  10      /* consecutive samples before acting (5 s) */
#define TUNE_LOW_PCT        30      /* below this % of capacity: we play too fast */
#define TUNE_HIGH_PCT       85      /* above this %: we play too slow */
#define TUNE_STEP_HZ        512     /* ~10 ppm of a 49.152 MHz MCLK per step */
#define TUNE_CLAMP_HZ       32768   /* +/-667 ppm total; crystals are well inside */

static int32_t  s_mclk_init_hz = 0;      /* 0 = tracking not armed */
static uint32_t s_tune_lo = 0, s_tune_hi = 0;
static int64_t  s_tune_next_us = 0;
static volatile uint64_t s_prod_bytes = 0;   /* accepted into the ring */
static volatile uint64_t s_full_bytes = 0;   /* refused, ring was full */
static uint64_t s_uf_bytes0 = 0;
static uint64_t s_uf_full0 = 0;
static int64_t  s_uf_t0 = 0;

static inline int32_t apply_gain_q15(int32_t s, int32_t g)
{
    int64_t v = ((int64_t)s * g) >> 15;
    if (v > INT32_MAX) return INT32_MAX;
    if (v < INT32_MIN) return INT32_MIN;
    return (int32_t)v;
}

/* Per-codec calibration gain (Q15). PLACEHOLDER: all unity == no change from
 * today. To equalize loudness: play the SAME reference track through each codec,
 * measure the analog output level, pick the QUIETEST codec as the 0 dB reference,
 * then set every other codec's entry to attenuate down to it, e.g.
 *   gain_q15 = (int32_t)(32768.0 * pow(10.0, -delta_dB/20.0))   // delta_dB >= 0
 * Keep all entries <= 32768 (attenuate only) so nothing clips. */
static int32_t codec_calib_gain_q15(const esp_a2d_mcc_t *mcc)
{
    if (mcc->type == ESP_A2D_MCT_SBC)  return A2DP_GAIN_Q15_UNITY;   /* TODO: measure */
    if (mcc->type == ESP_A2D_MCT_M24)  return A2DP_GAIN_Q15_UNITY;   /* AAC   TODO: measure */
    if (mcc->type == ESP_A2D_MCT_NON_A2DP) {
        const uint8_t *raw = (const uint8_t *)&mcc->cie;
        uint32_t vendor_id = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                             ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
        uint16_t codec_id  = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
        if (vendor_id == A2DP_LDAC_VENDOR_ID   && codec_id == A2DP_LDAC_CODEC_ID)   return A2DP_GAIN_Q15_UNITY; /* TODO */
        if (vendor_id == A2DP_APTX_VENDOR_ID   && codec_id == A2DP_APTX_CODEC_ID)   return A2DP_GAIN_Q15_UNITY; /* TODO */
        if (vendor_id == A2DP_APTXHD_VENDOR_ID && codec_id == A2DP_APTXHD_CODEC_ID) return A2DP_GAIN_Q15_UNITY; /* TODO */
        if (vendor_id == A2DP_APTXLL_VENDOR_ID && codec_id == A2DP_APTXLL_CODEC_ID) return A2DP_GAIN_Q15_UNITY; /* TODO */
        if (vendor_id == A2DP_OPUS_G_VENDOR_ID  && codec_id == A2DP_OPUS_G_CODEC_ID)  return A2DP_GAIN_Q15_UNITY; /* TODO */
        if (vendor_id == A2DP_OPUS_05_VENDOR_ID && codec_id == A2DP_OPUS_05_CODEC_ID) return A2DP_GAIN_Q15_UNITY; /* TODO */
        if (vendor_id == A2DP_LHDCV5_VENDOR_ID && codec_id == A2DP_LHDCV5_CODEC_ID) return A2DP_GAIN_Q15_UNITY; /* TODO */
    }
    return A2DP_GAIN_Q15_UNITY;
}

void audio_sink_srv_set_volume(uint8_t vol_0_127)
{
    if (vol_0_127 > 127) vol_0_127 = 127;
    /* Linear map for now; swap for a perceptual (e.g. squared) curve if desired. */
    s_volume_q15 = ((int32_t)vol_0_127 * A2DP_GAIN_Q15_UNITY) / 127;
}

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* task handler for writing data to i2s */
static void audio_sink_srv_i2s_task_handler(void *arg);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

/* audio sink service for i2s control block */
static audio_sink_srv_i2s_cb_t s_i2s_cb;

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

/* Sample the ring level and, on a sustained trend, nudge MCLK to follow the
 * source's clock. Runs ONLY from the render task, which owns tx_chan, so the
 * disable/enable pair cannot race another writer. */
static void audio_sink_srv_track_source_clock(void)
{
    if (s_mclk_init_hz <= 0 || s_i2s_cb.ringbuf == NULL) return;
    if (s_i2s_cb.chan_st != CHANNEL_STATUS_ENABLED) return;
    if (s_i2s_cb.ringbuffer_mode != RINGBUFFER_MODE_PROCESSING) return;  /* re-priming */

    const int64_t now = esp_timer_get_time();
    if (now < s_tune_next_us) return;
    s_tune_next_us = now + (int64_t)TUNE_PERIOD_MS * 1000;

    size_t fill = 0;
    vRingbufferGetInfo(s_i2s_cb.ringbuf, NULL, NULL, NULL, NULL, &fill);
    const uint32_t pct = (s_ring_capacity == 0) ? 50u
                       : (uint32_t)((uint64_t)fill * 100ull / s_ring_capacity);

    int dir = 0;
    if (pct < TUNE_LOW_PCT) {
        s_tune_hi = 0;
        if (++s_tune_lo >= TUNE_TREND_SAMPLES) dir = -1;   /* draining -> slow down */
    } else if (pct > TUNE_HIGH_PCT) {
        s_tune_lo = 0;
        if (++s_tune_hi >= TUNE_TREND_SAMPLES) dir = +1;   /* filling  -> speed up */
    } else {
        s_tune_lo = s_tune_hi = 0;                         /* inside the deadband */
    }
    if (dir == 0) return;
    s_tune_lo = s_tune_hi = 0;

    i2s_tuning_config_t cfg = {
        .tune_mode      = I2S_TUNING_MODE_ADDSUB,
        .tune_mclk_val  = dir * TUNE_STEP_HZ,
        .max_delta_mclk = TUNE_CLAMP_HZ,
        .min_delta_mclk = -TUNE_CLAMP_HZ,
    };
    i2s_tuning_info_t info = {0};
    /* Disable around the change: Espressif's guidance, and it also guarantees no
     * write is in flight while the divider moves. */
    if (i2s_channel_disable(s_i2s_cb.tx_chan) != ESP_OK) return;
    esp_err_t err = i2s_channel_tune_rate(s_i2s_cb.tx_chan, &cfg, &info);
    if (i2s_channel_enable(s_i2s_cb.tx_chan) != ESP_OK) {
        ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "i2s re-enable failed after clock tune");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "clock tune failed (%s) - tracking disabled", esp_err_to_name(err));
        s_mclk_init_hz = 0;              /* stop trying; behave as before */
        return;
    }
    /* delta in ppm of the ORIGINAL mclk tells us the source's actual offset. */
    const int32_t ppm = (int32_t)(((int64_t)info.delta_mclk_hz * 1000000ll) / s_mclk_init_hz);
    ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG,
             "clock track: ring %u%% -> mclk %+d Hz (now %d Hz, %+d ppm vs source)",
             (unsigned)pct, (int)(dir * TUNE_STEP_HZ),
             (int)info.curr_mclk_hz, (int)ppm);
}

static void audio_sink_srv_i2s_task_handler(void *arg)
{
    uint8_t *data = NULL;
    size_t item_size = 0;
    /**
     * The total length of DMA buffer of I2S is:
     * `dma_frame_num * dma_desc_num * i2s_channel_num * i2s_data_bit_width / 8`.
     * Transmit `dma_frame_num * dma_desc_num` bytes to DMA is trade-off.
     */
    const size_t item_size_upto = 240 * 6;
    size_t bytes_written = 0;

    for (;;) {
        if (pdTRUE == xSemaphoreTake(s_i2s_cb.write_semaphore, portMAX_DELAY)) {
            for (;;) {
                item_size = 0;
                /* receive data from ringbuffer and write it to I2S DMA transmit buffer */
                data = (uint8_t *)xRingbufferReceiveUpTo(s_i2s_cb.ringbuf, &item_size, (TickType_t)pdMS_TO_TICKS(20), item_size_upto);
                if (item_size == 0) {
                    {
                        /* Report the INPUT rate since the previous underflow, as a
                         * percentage of realtime. <100% means we are being starved
                         * (source or decoder); ~100% with drops means bursty delivery
                         * against too shallow a cushion. This is the number that says
                         * which of those it is, so print it every time. */
                        const int64_t now = esp_timer_get_time();
                        const uint64_t pb = s_prod_bytes, fb = s_full_bytes;
                        if (s_uf_t0 != 0 && now > s_uf_t0) {
                            const uint64_t dt_us = (uint64_t)(now - s_uf_t0);
                            const uint64_t want = (uint64_t)s_out_rate_hz * 8ull;   /* B/s */
                            const unsigned pct = (unsigned)(((pb - s_uf_bytes0) * 1000000ull * 100ull)
                                                            / (want * dt_us));
                            ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG,
                                     "underflow: input %u%% of realtime over %u ms (%u B in, %u B dropped-full)",
                                     pct, (unsigned)(dt_us / 1000u),
                                     (unsigned)(pb - s_uf_bytes0), (unsigned)(fb - s_uf_full0));
                        } else {
                            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "ringbuffer underflowed! mode changed: RINGBUFFER_MODE_PREFETCHING");
                        }
                        s_uf_t0 = now; s_uf_bytes0 = pb; s_uf_full0 = fb;
                    }
                    s_i2s_cb.ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
                    break;
                }

                if (s_i2s_cb.chan_st == CHANNEL_STATUS_ENABLED) {
                    i2s_channel_write(s_i2s_cb.tx_chan, data, item_size, &bytes_written, portMAX_DELAY);
                }
                vRingbufferReturnItem(s_i2s_cb.ringbuf, (void *)data);
                audio_sink_srv_track_source_clock();
            }
        }
    }
}

/*******************************
 * EXTERNAL FUNCTION DEFINITIONS
 ******************************/

void audio_sink_srv_open(void)
{
    if (s_i2s_cb.chan_st != CHANNEL_STATUS_IDLE) {
        ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "Service already open, skipping initialization");
        return;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;              /* play silence (not stale data) on underrun */
    chan_cfg.dma_desc_num = 6;               /* match old project: 6 x 128 = 768-frame DMA */
    chan_cfg.dma_frame_num = 128;            /* ~8 ms @96k / ~17 ms @48k render bridge */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = CONFIG_EXAMPLE_I2S_BCK_PIN,
            .ws = CONFIG_EXAMPLE_I2S_LRCK_PIN,
            .dout = CONFIG_EXAMPLE_I2S_DATA_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    /* initialize I2S channel */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_cb.tx_chan, NULL));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_cb.tx_chan, &std_cfg));
    s_i2s_cb.chan_st = CHANNEL_STATUS_OPENED;
}

void audio_sink_srv_close(void)
{
    audio_sink_srv_stop();

    if (s_i2s_cb.write_task_handle) {
        vTaskDelete(s_i2s_cb.write_task_handle);
        s_i2s_cb.write_task_handle = NULL;
    }
    if (s_i2s_cb.ringbuf) {
        RING_DELETE(s_i2s_cb.ringbuf);
        s_i2s_cb.ringbuf = NULL;
    }
    if (s_i2s_cb.write_semaphore) {
        vSemaphoreDelete(s_i2s_cb.write_semaphore);
        s_i2s_cb.write_semaphore = NULL;
    }
    if (s_i2s_cb.chan_st == CHANNEL_STATUS_OPENED) {
        ESP_ERROR_CHECK(i2s_del_channel(s_i2s_cb.tx_chan));
        s_i2s_cb.chan_st = CHANNEL_STATUS_IDLE;
    }
    memset(&s_i2s_cb, 0, sizeof(audio_sink_srv_i2s_cb_t));
}

void audio_sink_srv_start(void)
{
    if (s_i2s_cb.chan_st == CHANNEL_STATUS_ENABLED) {
        return;  /* already streaming (e.g. AUDIO_STATE STARTED right after a fresh connect) */
    }
    if (s_i2s_cb.chan_st != CHANNEL_STATUS_OPENED) {
        ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "%s, TX channel wrong state: %d", __func__, s_i2s_cb.chan_st);
        return;
    }
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_cb.tx_chan));

    /* Arm source-clock tracking: remember the MCLK the driver actually achieved
     * (it is the closest the APLL can synthesize, not necessarily the ideal
     * rate * mclk_multiple) so every later adjustment is relative to it and the
     * +/-clamp is anchored somewhere meaningful. Failure here is not fatal --
     * s_mclk_init_hz stays 0 and the loop simply never runs. */
    {
        i2s_tuning_info_t info = {0};
        if (i2s_channel_tune_rate(s_i2s_cb.tx_chan, NULL, &info) == ESP_OK) {
            s_mclk_init_hz = info.curr_mclk_hz;
            const int32_t ideal = (int32_t)s_out_rate_hz * 256;   /* MCLK_MULTIPLE_256 */
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG,
                     "clock track armed: mclk %d Hz (ideal %d, %+d ppm), ring target %u-%u%%",
                     (int)s_mclk_init_hz, (int)ideal,
                     (int)(ideal ? (((int64_t)(s_mclk_init_hz - ideal) * 1000000ll) / ideal) : 0),
                     (unsigned)TUNE_LOW_PCT, (unsigned)TUNE_HIGH_PCT);
        } else {
            s_mclk_init_hz = 0;
            ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "clock tracking unavailable on this clock source");
        }
        s_tune_lo = s_tune_hi = 0;
        s_tune_next_us = 0;
    }

    ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "ringbuffer data empty! mode changed: RINGBUFFER_MODE_PREFETCHING");
    s_i2s_cb.ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    if ((s_i2s_cb.write_semaphore == NULL) && (s_i2s_cb.write_semaphore = xSemaphoreCreateBinary()) == NULL) {
        ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "%s, Semaphore create failed", __func__);
        goto err_sem;
    }
    if (s_i2s_cb.ringbuf == NULL) {
#if RING_IN_PSRAM
        /* PSRAM is plentiful -- take the full ceiling directly for a big output
         * cushion (128 KB = ~85 ms @192k) to ride out decode-spike stalls. */
        s_i2s_cb.ringbuf = xRingbufferCreateWithCaps(RINGBUF_HIGHEST_WATER_LEVEL, RINGBUF_TYPE_BYTEBUF,
                                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_i2s_cb.ringbuf != NULL) {
            s_ring_capacity = RINGBUF_HIGHEST_WATER_LEVEL;
            { uint32_t pfmax = (s_ring_capacity >> 2) * RINGBUF_PREFETCH_NUM;
              if (s_prefetch_bytes > pfmax) s_prefetch_bytes = pfmax; }
        }
#else
        /* Grab the largest contiguous BYTEBUF internal RAM can give, from the ceiling
         * down. S31 lands on the full 128 KB; a PSRAM-off classic ESP32 (largest block
         * ~108 KB) falls back to the biggest that fits. */
        /* Floor/step scale with the ceiling: a hardcoded 32 KB floor would skip the
         * loop entirely (-> ringbuf NULL -> no audio) once the ceiling is trimmed
         * below it. Try the ceiling first, then step down to a hard 8 KB minimum. */
        uint32_t ring_top = RINGBUF_HIGHEST_WATER_LEVEL;
        /* "Largest block fits" is NOT the constraint that matters here: the
         * allocation can succeed and still leave the BT stack with nothing. Cap the
         * starting size at what byte-DRAM can spare on top of RINGBUF_BT_DRAM_RESERVE,
         * measured NOW -- after the decoder has already claimed its rate-sized work
         * buffers, so a 192k config automatically gets a smaller ring than a 48k one
         * without any rate table here. */
        {
            size_t free8 = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
            uint32_t spare = (free8 > RINGBUF_BT_DRAM_RESERVE)
                           ? (uint32_t)(free8 - RINGBUF_BT_DRAM_RESERVE) : 0;
            spare &= ~((uint32_t)4095);           /* round down to 4 KB */
            if (ring_top > spare) ring_top = spare;
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG,
                     "output ring budget: 8-bit DRAM free %u KB - %u KB BT reserve -> target %u KB (ceiling %u KB)",
                     (unsigned)(free8 / 1024), (unsigned)(RINGBUF_BT_DRAM_RESERVE / 1024),
                     (unsigned)(ring_top / 1024), (unsigned)(RINGBUF_HIGHEST_WATER_LEVEL / 1024));
        }
        /* A ring is mandatory -- with none there is no audio at all -- so never let
         * the budget drive the target to zero. 8 KB is the hard floor. */
        if (ring_top < (8 * 1024)) ring_top = 8 * 1024;
        uint32_t ring_min = 8 * 1024;
        uint32_t ring_step = RINGBUF_HIGHEST_WATER_LEVEL / 8;
        if (ring_step < (2 * 1024)) ring_step = 2 * 1024;
        for (uint32_t cap = ring_top; cap >= ring_min && cap > 0;
             cap = (cap > ring_step) ? (cap - ring_step) : 0) {
            s_i2s_cb.ringbuf = xRingbufferCreate(cap, RINGBUF_TYPE_BYTEBUF);
            if (s_i2s_cb.ringbuf != NULL) {
                s_ring_capacity = cap;
                /* Keep the re-prime level <= half the ACTUAL ring, even if it was
                 * computed against the default ceiling before we knew the real size. */
                { uint32_t pfmax = (s_ring_capacity >> 2) * RINGBUF_PREFETCH_NUM;
              if (s_prefetch_bytes > pfmax) s_prefetch_bytes = pfmax; }
                if (cap != RINGBUF_HIGHEST_WATER_LEVEL) {
                    ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "output ring %u KB (budget %u KB, ceiling %u KB)",
                             (unsigned)(cap / 1024), (unsigned)(ring_top / 1024),
                             (unsigned)(RINGBUF_HIGHEST_WATER_LEVEL / 1024));
                }
                break;
            }
        }
#endif
        if (s_i2s_cb.ringbuf == NULL) {
            ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "%s, ringbuffer create failed", __func__);
            goto err_rb;
        }
    }
    if (s_i2s_cb.write_task_handle == NULL) {
        /* Pin the I2S render task to core 0 (with the BT controller + host); the
         * heavy A2DP decode owns core 1, so this light task must not land there. */
        if (xTaskCreatePinnedToCore(audio_sink_srv_i2s_task_handler, "BtI2STask", 4 * 1024, NULL,
                        configMAX_PRIORITIES - 3, &s_i2s_cb.write_task_handle, 0) != pdPASS) {
            ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "%s, Task create failed", __func__);
            goto err_task;
        }
    }
    s_i2s_cb.chan_st = CHANNEL_STATUS_ENABLED;
    return;

err_task:
    RING_DELETE(s_i2s_cb.ringbuf);
    s_i2s_cb.ringbuf = NULL;
err_rb:
    vSemaphoreDelete(s_i2s_cb.write_semaphore);
    s_i2s_cb.write_semaphore = NULL;
err_sem:
    i2s_channel_disable(s_i2s_cb.tx_chan);
}

void audio_sink_srv_stop(void)
{
    if (s_i2s_cb.chan_st == CHANNEL_STATUS_ENABLED) {
        /* Stop I2S output immediately; auto_clear zeros the DMA so no residual plays. */
        i2s_channel_disable(s_i2s_cb.tx_chan);
        s_i2s_cb.chan_st = CHANNEL_STATUS_OPENED;
    }
    /* Clear the audio still buffered so nothing lingers/plays on the next stream.
     * The write task no longer outputs (channel disabled); drain and discard the
     * ring here, then reset the fill state to prefetch fresh audio. */
    if (s_i2s_cb.ringbuf) {
        size_t item_size = 0;
        void *item;
        while ((item = xRingbufferReceiveUpTo(s_i2s_cb.ringbuf, &item_size, 0, RINGBUF_HIGHEST_WATER_LEVEL)) != NULL) {
            vRingbufferReturnItem(s_i2s_cb.ringbuf, item);
        }
    }
    s_i2s_cb.ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
    ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "I2S output stopped, buffer cleared");
}

void audio_sink_srv_codec_info_update(esp_a2d_mcc_t *mcc)
{
    /* Codec-switch leak accounting, second half (the first half is the
     * "DRAM decoder_reset" line from btc_a2dp_sink). This covers the I2S slot +
     * clock reconfiguration, which is the other thing a codec switch touches. */
    size_t dram_in = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "A2DP audio stream configuration, codec type: %d", mcc->type);

    /* On a codec switch the peer briefly disconnects, which closes/deletes the
     * I2S channel (tx_chan == NULL). The new codec's AUDIO_CFG can arrive before
     * the channel is re-opened, so recreate it here — otherwise the reconfig
     * calls below get a NULL handle and abort. */
    if (s_i2s_cb.tx_chan == NULL) {
        audio_sink_srv_open();
    }
    if (s_i2s_cb.tx_chan == NULL) {
        ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "I2S channel unavailable, skipping codec config");
        return;
    }
    audio_sink_srv_stop();

    /* Determine sample rate, decoder output bit depth and channel count for
     * every codec (SBC/AAC + vendor codecs), then configure I2S to match.
     * Mirrors the old bt_audio_sink project's per-codec I2S setup. */
    int sample_rate = 44100;
    int ch_count = 2;
    int bits = 16;              /* decoder PCM width: 16 (SBC/AAC/Opus), 24 (LDAC), 32 (aptX/LHDC) */

    if (mcc->type == ESP_A2D_MCT_SBC) {
        if (mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_32K)      sample_rate = 32000;
        else if (mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_44K) sample_rate = 44100;
        else if (mcc->cie.sbc_info.samp_freq & ESP_A2D_SBC_CIE_SF_48K) sample_rate = 48000;
        else                                                          sample_rate = 16000;
        if (mcc->cie.sbc_info.ch_mode & ESP_A2D_SBC_CIE_CH_MODE_MONO)  ch_count = 1;
        bits = 16;
    } else if (mcc->type == ESP_A2D_MCT_M24) {
        if (mcc->cie.m24_info.samp_freq2 & ESP_A2D_M24_CIE_SF2_96K)      sample_rate = 96000;
        else if (mcc->cie.m24_info.samp_freq2 & ESP_A2D_M24_CIE_SF2_88K) sample_rate = 88200;
        else if (mcc->cie.m24_info.samp_freq2 & ESP_A2D_M24_CIE_SF2_64K) sample_rate = 64000;
        else if (mcc->cie.m24_info.samp_freq2 & ESP_A2D_M24_CIE_SF2_48K) sample_rate = 48000;
        else if (mcc->cie.m24_info.samp_freq1 & ESP_A2D_M24_CIE_SF1_44K) sample_rate = 44100;
        else if (mcc->cie.m24_info.samp_freq1 & ESP_A2D_M24_CIE_SF1_32K) sample_rate = 32000;
        else if (mcc->cie.m24_info.samp_freq1 & ESP_A2D_M24_CIE_SF1_24K) sample_rate = 24000;
        else if (mcc->cie.m24_info.samp_freq1 & ESP_A2D_M24_CIE_SF1_22K) sample_rate = 22050;
        else if (mcc->cie.m24_info.samp_freq1 & ESP_A2D_M24_CIE_SF1_16K) sample_rate = 16000;
        else                                                            sample_rate = 44100;
        if (mcc->cie.m24_info.ch & ESP_A2D_M24_CIE_CH_1) ch_count = 1;
        bits = 16;
    } else if (mcc->type == ESP_A2D_MCT_NON_A2DP) {
        const uint8_t *raw = (const uint8_t *)&mcc->cie;   /* vendor CIE bytes */
        uint32_t vendor_id = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                             ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
        uint16_t codec_id  = (uint16_t)raw[4] | ((uint16_t)raw[5] << 8);
        if (vendor_id == A2DP_LDAC_VENDOR_ID && codec_id == A2DP_LDAC_CODEC_ID) {
            uint8_t sf = raw[6] & 0x3F;
            if (sf & 0x20)      sample_rate = 44100;
            else if (sf & 0x10) sample_rate = 48000;
            else if (sf & 0x08) sample_rate = 88200;
            else if (sf & 0x04) sample_rate = 96000;
            else if (sf & 0x02) sample_rate = 176400;
            else if (sf & 0x01) sample_rate = 192000;
            if ((raw[7] & 0x07) == 0x04) ch_count = 1;
            bits = 24;      /* LDACBT decoder outputs S24 */
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: LDAC");
        } else if (vendor_id == A2DP_APTX_VENDOR_ID && codec_id == A2DP_APTX_CODEC_ID) {
            if (raw[6] & 0x20)      sample_rate = 44100;
            else if (raw[6] & 0x10) sample_rate = 48000;
            if ((raw[6] & 0x0F) == 0x01) ch_count = 1;
            bits = 32;      /* 24-bit in 32-bit LE containers */
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: aptX");
        } else if (vendor_id == A2DP_APTXHD_VENDOR_ID && codec_id == A2DP_APTXHD_CODEC_ID) {
            if (raw[6] & 0x20)      sample_rate = 44100;
            else if (raw[6] & 0x10) sample_rate = 48000;
            if ((raw[6] & 0x0F) == 0x01) ch_count = 1;
            bits = 32;
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: aptX-HD");
        } else if (vendor_id == A2DP_APTXLL_VENDOR_ID && codec_id == A2DP_APTXLL_CODEC_ID) {
            sample_rate = 48000; bits = 32;
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: aptX-LL");
        } else if ((vendor_id == A2DP_OPUS_G_VENDOR_ID  && codec_id == A2DP_OPUS_G_CODEC_ID) ||
                   (vendor_id == A2DP_OPUS_05_VENDOR_ID && codec_id == A2DP_OPUS_05_CODEC_ID)) {
            bool opus_g = (vendor_id == A2DP_OPUS_G_VENDOR_ID);
            sample_rate = 48000; bits = 16;   /* Opus decoder outputs 48k/16-bit */
            /* Channel mode lives at raw[6] in the 9-byte Google CIE; the 26-byte
             * Opus-05 CIE has a different layout, so don't read it there. */
            if (opus_g) ch_count = (raw[6] == 1) ? 1 : 2;
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: Opus (%s)",
                     opus_g ? "Google" : "05");
        } else if (vendor_id == A2DP_LHDCV5_VENDOR_ID && codec_id == A2DP_LHDCV5_CODEC_ID) {
            uint8_t sf = raw[6] & 0x35;
            if (sf & 0x01)      sample_rate = 192000;
            else if (sf & 0x04) sample_rate = 96000;
            else if (sf & 0x10) sample_rate = 48000;
            else if (sf & 0x20) sample_rate = 44100;
            bits = ((raw[7] & 0x04)) ? 16 : 32;   /* 24-bit source up-packed to 32-bit container */
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: LHDC-v5");
        } else if (vendor_id == A2DP_LC3PLUS_VENDOR_ID && codec_id == A2DP_LC3PLUS_CODEC_ID) {
            /* frequency is a 16-bit field at raw[6..7] (hi,lo): 48k=1<<8 (raw[6]&0x01),
             * 96k=1<<7 (raw[7]&0x80). Decoder emits S24-in-int32, left-shifted <<8 in
             * a2dp_vendor_lc3plus_decoder.c so it is left-justified -> bits=32 passthrough. */
            if (raw[6] & 0x01)      sample_rate = 48000;
            else if (raw[7] & 0x80) sample_rate = 96000;
            else                    sample_rate = 48000;
            bits = 32;
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "Vendor codec: LC3plus");
        } else {
            sample_rate = 48000; bits = 32;
            ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "Unknown vendor codec vid=0x%08x cid=0x%04x", vendor_id, codec_id);
        }
    }

    /* Old-project model: I2S is ALWAYS 32-bit stereo; only the sample rate
     * changes per codec. The decoder's native PCM (16/24/32-bit) is up-converted
     * to 32-bit in audio_sink_srv_data_output(). Using 32-bit slots (256x MCLK)
     * avoids the 24-bit "sample rate too large" (384x MCLK) failure at 96 kHz. */
    s_i2s_cb.cur_bits = (uint8_t)bits;
    s_out_rate_hz = (uint32_t)sample_rate;
    s_prod_bytes = s_full_bytes = 0;
    s_uf_bytes0 = s_uf_full0 = 0;
    s_uf_t0 = 0;
    s_i2s_cb.cur_calib_q15 = codec_calib_gain_q15(mcc);   /* unified-loudness calibration */

    /* Rate-proportional prefetch: a fixed TIME cushion at the actual output rate,
     * because 192k drains 4x faster than 48k for the same byte count. I2S is
     * fixed 32-bit stereo = 8 bytes/frame.
     *
     * Raised 42 -> 80 ms. 42 ms was tuned against a phone on a direct radio; a
     * source behind a VM's USB-passthrough Bluetooth delivers in coarser bursts,
     * which showed up as continuous "ringbuffer underflowed -> PREFETCHING ->
     * PROCESSING" cycling every ~60 ms on LDAC at 48k. Buffering costs only
     * latency (the sink already reports 270 ms) and it only helps JITTER -- if
     * the source is short on average throughput no cushion fixes it, and the
     * symptom is underflow that continues at any depth.
     * Clamp: floor at the compile-time default, ceiling at half the ring so there
     * is always burst headroom above the re-prime level. */
    {
        uint32_t want = ((uint32_t)sample_rate * 8u * 80u) / 1000u;   /* bytes for ~80 ms */
        if (want < RINGBUF_PREFETCH_WATER_LEVEL) want = RINGBUF_PREFETCH_WATER_LEVEL;
        { uint32_t pfmax = (s_ring_capacity >> 2) * RINGBUF_PREFETCH_NUM;   /* actual ring, not the macro */
          if (want > pfmax) want = pfmax; }
        s_prefetch_bytes = want;
    }

    i2s_std_slot_config_t slot_cfg =
        I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
    ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(s_i2s_cb.tx_chan, &slot_cfg));

    /* High sample rates (LDAC 96k, LHDC 192k) need the Audio PLL; the default
     * I2S clock source cannot synthesize them ("sample rate too large").
     * Try APLL first, fall back to default PLL. Never ESP_ERROR_CHECK here: a
     * rate the hardware cannot produce must not crash the device. */
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG((uint32_t)sample_rate);
    clk_cfg.clk_src = I2S_CLK_SRC_APLL;
    clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    esp_err_t clk_err = i2s_channel_reconfig_std_clock(s_i2s_cb.tx_chan, &clk_cfg);
    if (clk_err != ESP_OK) {
        ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "APLL clock @%d Hz failed (%s), trying default PLL",
                 sample_rate, esp_err_to_name(clk_err));
        clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
        clk_err = i2s_channel_reconfig_std_clock(s_i2s_cb.tx_chan, &clk_cfg);
        if (clk_err != ESP_OK) {
            ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "I2S clock @%d Hz unsupported (%s)",
                     sample_rate, esp_err_to_name(clk_err));
        }
    }

    ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG,
             "I2S CONFIG => Fs(LRCK)=%d Hz | decoder=%d-bit -> I2S 32-bit | ch=%d | fmt=Philips | role=MASTER",
             sample_rate, bits, ch_count);
    ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG,
             "I2S PINS   => BCLK=GPIO%d  WS/LRCK=GPIO%d  DOUT=GPIO%d  MCLK=unused",
             CONFIG_EXAMPLE_I2S_BCK_PIN, CONFIG_EXAMPLE_I2S_LRCK_PIN, CONFIG_EXAMPLE_I2S_DATA_PIN);
    {
        size_t dram_out = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "DRAM i2s_reconfig: in=%u out=%u delta=%+d B",
                 (unsigned)dram_in, (unsigned)dram_out,
                 (int)((long)dram_out - (long)dram_in));
    }
}

size_t audio_sink_srv_data_output(const uint8_t *data, size_t size)
{
    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    if (s_i2s_cb.ringbuf == NULL) {
        return 0;
    }

    /* Up-convert the decoder's native PCM to 32-bit for the fixed 32-bit I2S
     * slots (matches the old project, which always ran 32-bit). A single fixed
     * static scratch buffer (no per-packet malloc/free) keeps this on internal
     * RAM without heap churn/fragmentation; larger packets are chunked. Called
     * only from the single A2DP sink data task, so the static buffer is safe. */
    static int32_t s_conv[1024];                    /* 4 KB scratch, 1024 samples */
    const size_t chunk_samples = sizeof(s_conv) / sizeof(s_conv[0]);
    int bits = s_i2s_cb.cur_bits ? s_i2s_cb.cur_bits : 16;
    /* Combine per-codec calibration with live global volume ONCE per packet, so a
     * runtime volume change takes effect immediately (not only at next reconfig). */
    int32_t calib = s_i2s_cb.cur_calib_q15 ? s_i2s_cb.cur_calib_q15 : A2DP_GAIN_Q15_UNITY;
    int32_t gain  = (int32_t)(((int64_t)calib * s_volume_q15) >> 15);
    done = pdTRUE;
    if (bits == 32 && gain == A2DP_GAIN_Q15_UNITY) {
        /* Fast path: already 32-bit at unity gain -> zero-copy straight to ring.
         * This is the LHDC-192k/aptX path; it stays free of the gain multiply. */
        done = xRingbufferSend(s_i2s_cb.ringbuf, (void *)data, size, (TickType_t)pdMS_TO_TICKS(RINGBUF_SEND_WAIT_MS));
    } else if (bits == 16) {
        const int16_t *in = (const int16_t *)data;
        size_t total = size >> 1, off = 0;
        while (off < total) {
            size_t n = total - off; if (n > chunk_samples) n = chunk_samples;
            for (size_t i = 0; i < n; i++) s_conv[i] = apply_gain_q15(((int32_t)in[off + i]) << 16, gain);
            if (!xRingbufferSend(s_i2s_cb.ringbuf, s_conv, n * sizeof(int32_t), (TickType_t)pdMS_TO_TICKS(RINGBUF_SEND_WAIT_MS))) { done = pdFALSE; break; }
            off += n;
        }
    } else if (bits == 24) {                        /* 24-bit, 3-byte packed LE */
        size_t total = size / 3, off = 0;
        while (off < total) {
            size_t n = total - off; if (n > chunk_samples) n = chunk_samples;
            for (size_t i = 0; i < n; i++) {
                const uint8_t *p = data + (off + i) * 3;
                int32_t s = (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
                if (s & 0x00800000) s |= (int32_t)0xFF000000;   /* sign-extend 24 -> 32 */
                s_conv[i] = apply_gain_q15(s << 8, gain);       /* left-justify in 32-bit slot + gain */
            }
            if (!xRingbufferSend(s_i2s_cb.ringbuf, s_conv, n * sizeof(int32_t), (TickType_t)pdMS_TO_TICKS(RINGBUF_SEND_WAIT_MS))) { done = pdFALSE; break; }
            off += n;
        }
    } else {                                        /* 32-bit source with non-unity gain */
        const int32_t *in = (const int32_t *)data;
        size_t total = size >> 2, off = 0;
        while (off < total) {
            size_t n = total - off; if (n > chunk_samples) n = chunk_samples;
            for (size_t i = 0; i < n; i++) s_conv[i] = apply_gain_q15(in[off + i], gain);
            if (!xRingbufferSend(s_i2s_cb.ringbuf, s_conv, n * sizeof(int32_t), (TickType_t)pdMS_TO_TICKS(RINGBUF_SEND_WAIT_MS))) { done = pdFALSE; break; }
            off += n;
        }
    }

    if (done) {
        s_prod_bytes += (uint64_t)size;
    } else {
        s_full_bytes += (uint64_t)size;
    }

    if (!done) {
        /* Buffer full: the phone is streaming ahead of realtime. Drop ONLY this
         * chunk (graceful, bounded) instead of bulk-draining the whole cushion,
         * which is what caused the audible gaps. The cushion stays deep so the
         * I2S render never starves; only the source's realtime surplus is shed. */
        static uint32_t s_drop_count = 0;
        if ((++s_drop_count & 0x1FF) == 0) {
            ESP_LOGW(AUDIO_SNK_SRV_I2S_TAG, "source ahead of realtime, dropped %u chunks", (unsigned)s_drop_count);
        }
    }

    if (s_i2s_cb.ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_i2s_cb.ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= s_prefetch_bytes) {
            ESP_LOGI(AUDIO_SNK_SRV_I2S_TAG, "ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING");
            s_i2s_cb.ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
            if (pdFALSE == xSemaphoreGive(s_i2s_cb.write_semaphore)) {
                ESP_LOGE(AUDIO_SNK_SRV_I2S_TAG, "semaphore give failed");
            }
        }
    }

    return done ? size : 0;
}

#endif /* defined(CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_EXTERNAL_I2S) */
