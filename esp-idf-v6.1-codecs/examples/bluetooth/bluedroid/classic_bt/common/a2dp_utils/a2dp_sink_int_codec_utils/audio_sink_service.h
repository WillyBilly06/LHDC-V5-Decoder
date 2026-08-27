/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#ifndef __AUDIO_SINK_SERVICE_H__
#define __AUDIO_SINK_SERVICE_H__

#include <stdint.h>
#include <string.h>
#include "esp_a2dp_api.h"

/* Sized for the fixed 32-bit I2S path (2x the byte-rate of the original 16-bit
 * util) with a deep cushion to absorb phones that stream ahead of realtime AND
 * the bursty per-frame delivery of hi-res codecs (LHDC frames fragment across
 * several A2DP packets and emit PCM only on the final fragment).
 * Measured on-device (2026-07-21): steady 24-bit LHDC playback at 48k/192k is
 * clean; underflows occurred ONLY at suspend/rate-switch reconfigs, where the
 * old 16 KB re-prime oscillated (4 underflows in 2.6 s) before restabilizing.
 * Deeper ring + deeper prefetch make the post-reconfig refill settle in one shot.
 * The prefetch (re-prime / steady cushion) is now RATE-PROPORTIONAL and set at
 * runtime from the sample rate (see audio_sink_service_i2s.c) — a fixed BYTE
 * count gives 4x less TIME at 192k than at 48k, which is exactly why only 192k
 * underflowed. The runtime prefetch targets a constant ~42 ms at every rate:
 * 16 KB @48k (unchanged) → 32 KB @96k → 64 KB @192k. RINGBUF_PREFETCH_WATER_LEVEL
 * below is only the compile-time DEFAULT/floor. The ceiling is sized to hold the
 * 192k prefetch (64 KB) plus burst headroom; the ceiling costs no latency (only
 * the prefetch level does), so 128 KB is safe (device: 291 KB free / 256 KB
 * largest block). 128 KB ≈ 85 ms @192k / 341 ms @48k. */
/* The ring storage is a plain malloc() = MALLOC_CAP_DEFAULT (byte-addressable
 * DRAM) -- the SAME pool the BT media-packet allocator draws from. On the classic
 * ESP32 that pool is only ~82 KB (the rest of SRAM is 32-bit-only IRAM that BT
 * can't use), and the mandatory LHDC 192k workspace (32.5 KB) already lives there,
 * so a 64 KB ring starves the BT pool at 192k -> calloc failures -> drops. Cap the
 * ring lower on the classic ESP32 so ring + workspace + BT pool all fit; other
 * targets (e.g. S31, uniform byte-addressable SRAM + more headroom) keep 128 KB. */
#if CONFIG_IDF_TARGET_ESP32
/* Classic ESP32 (PSRAM-off): the ring is internal byte-DRAM (same pool as the BT
 * packet buffers + LHDC workspace), so it used to be held at 32 KB.
 *
 * That number was set for a WROVER, whose WHOLE byte-DRAM pool is ~82 KB. The
 * board this now runs on is a WROOM (ESP32-D0WD-V3, no PSRAM) reporting ~90 KB
 * of byte-DRAM FREE at idle, so the constraint that justified 32 KB no longer
 * binds. 32 KB was also actively hurting: the prefetch is rate-proportional but
 * clamped to half the ring, so 96k got 21 ms of cushion and 192k got 10.7 ms
 * instead of the intended 42 ms.
 *
 * MEASURED, not assumed: 64 KB was tried and is too much on this board. The
 * ~90 KB figure is at IDLE; once connected (BT pools + decoder workspace + the
 * 20 KB A2DP_DECODER stack) a 64 KB ring left only **9.8 KB** of byte-DRAM free
 * with an 8 KB largest block -- the exact shape that produces
 * "BT_OSI: calloc failed (size=535)" under load. 48 KB leaves ~26 KB, close to
 * the ~32 KB the WROVER ran cleanly at, and still lifts the 48k prefetch from
 * 42.7 ms to 64 ms and unclamps more of 96k.
 *
 * The ring create loop steps down if the allocation fails, so a tighter board
 * degrades instead of breaking. Watch "8-bit DRAM free" in the heap monitor --
 * it must stay well clear of the ~535 B BT media-packet allocation size. */
#define RINGBUF_HIGHEST_WATER_LEVEL    (48 * 1024)
#else
#define RINGBUF_HIGHEST_WATER_LEVEL    (128 * 1024)
#endif
#define RINGBUF_PREFETCH_WATER_LEVEL   (16 * 1024)

typedef enum {
    RINGBUFFER_MODE_PROCESSING,    /* ringbuffer is buffering incoming audio data */
    RINGBUFFER_MODE_PREFETCHING,   /* ringbuffer is buffering incoming audio data */
    RINGBUFFER_MODE_DROPPING       /* ringbuffer is not buffering (dropping) incoming audio data */
} audio_sink_ringbuffer_mode_t;

typedef enum {
    CHANNEL_STATUS_IDLE,
    CHANNEL_STATUS_OPENED,
    CHANNEL_STATUS_ENABLED
} audio_sink_chan_st_t;

/**
 * @brief  open audio sink service
 */
void audio_sink_srv_open(void);

/**
 * @brief  close audio sink service
 */
void audio_sink_srv_close(void);

/**
 * @brief  start audio sink service
 */
void audio_sink_srv_start(void);

/**
 * @brief  stop audio sink service
 */
void audio_sink_srv_stop(void);

/**
 * @brief  update codec information
 *
 * @param [in] mcc  codec information
 */
void audio_sink_srv_codec_info_update(esp_a2d_mcc_t *mcc);

/**
 * @brief  write data to ringbuffer
 *
 * @param [in] data  pointer to data stream
 * @param [in] size  data length in byte
 *
 * @return size if written ringbuffer successfully, 0 others
 */
size_t audio_sink_srv_data_output(const uint8_t *data, size_t size);

/**
 * @brief  Set the global output volume (applied uniformly across all codecs).
 *
 * Multiplies the per-codec loudness calibration, so the resulting level is
 * codec-independent. Takes effect immediately on the next decoded packet.
 *
 * @param [in] vol_0_127  volume, 0 (mute) .. 127 (max)
 */
void audio_sink_srv_set_volume(uint8_t vol_0_127);

#endif /* __AUDIO_SINK_SERVICE_H__ */
