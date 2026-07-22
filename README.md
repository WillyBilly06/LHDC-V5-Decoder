# LHDC V5 Decoder

A from-scratch software decoder for the **LHDC V5** Bluetooth audio codec, written in
portable C and tuned to run in real time on the **ESP32 (Xtensa LX6)** as part of a
Bluedroid **A2DP sink**. It takes the raw LHDC V5 frame stream delivered over A2DP and
reconstructs interleaved stereo PCM.

> **Note on LHDC:** LHDC is a proprietary high-resolution Bluetooth codec developed by
> Savitech. This repository is an **independent decoder implementation** for
> interoperability/research. It contains no Savitech/Qualcomm binaries. You are
> responsible for ensuring you have the rights to use LHDC in your jurisdiction and
> product. The round-trip test tool additionally requires the proprietary `liblhdcv5.so`
> encoder, which is **not** included here (see [Testing](#testing)).

## Table of contents

1. [Features](#features)
2. [Hardware requirements](#hardware-requirements)
3. [Repository layout](#repository-layout)
4. [Add to your ESP-IDF project](#add-to-your-esp-idf-project)
5. [Quick start](#quick-start)
6. [Decoder API reference](#decoder-api-reference)
7. [Data types](#data-types)
8. [Memory model](#memory-model)
9. [How the decode pipeline works](#how-the-decode-pipeline-works)
10. [Integrating into an ESP-IDF A2DP sink](#integrating-into-an-esp-idf-a2dp-sink)
11. [192 kHz / PSRAM build](#192-khz--psram-build)
12. [Testing](#testing)
13. [Verification results](#verification-results)
14. [Performance & Xtensa tuning](#performance--xtensa-tuning)
15. [Diagnostics](#diagnostics)
16. [Troubleshooting](#troubleshooting)
17. [Limitations](#limitations)
18. [License](#license)

---

## Features

- **Sample rates:** 44.1 kHz, 48 kHz, 96 kHz, 192 kHz
- **Bit depths:** 16-bit and 24-bit (24-bit is emitted in a 32-bit-aligned container)
- **Target bitrates:** 256, 400, 500, 900 kbps, and Auto Bit Rate
- **Channels:** stereo (direct L/R) and mono
- **Frame duration:** 5 ms
- Fully float pipeline: FAC range decoder -> Rice -> mantissa bit-plane -> SNS synthesis ->
  inverse quantization -> fast LD-IMDCT -> windowed overlap-add
- Tuned for Xtensa LX6: hot paths in IRAM, single-precision FPU throughout, no integer
  divide in the range-coder inner loop, FFT-based IMDCT (no O(N^2) transform)
- Self-contained: the decoder math has no ESP-IDF dependency and builds on a host PC too

### Real-time support matrix

| Sample rate | 16-bit | 24-bit | Real-time on ESP32 | MDCT size |
|-------------|:------:|:------:|--------------------|:---------:|
| 44.1 kHz    | yes    | yes    | yes, single core    | 480 |
| 48 kHz      | yes    | yes    | yes, single core    | 480 |
| 96 kHz      | yes    | yes    | yes, single core    | 960 |
| 192 kHz     | yes    | yes    | requires PSRAM + dual-core (experimental) | 1920 |

All five bitrate modes (256/400/500/900 kbps + Auto) decode correctly at every sample
rate; real-time playback is solid at 44.1/48/96 kHz.

---

## Hardware requirements

- **ESP32** (classic, dual-core LX6), **ESP-IDF 5.x**.
- For **192 kHz**: a module with **PSRAM** (e.g. **ESP32-WROVER**, N8R8). See
  [192 kHz / PSRAM build](#192-khz--psram-build).
- No PSRAM needed for 44.1 / 48 / 96 kHz.
- The decoder math is portable C; it also compiles on a host (Linux/Android) for testing
  via `-DLHDC_HOST_BUILD`.

---

## Repository layout

```
CMakeLists.txt       ESP-IDF component manifest — makes this repo a drop-in component
idf_component.yml    Component-manager metadata (version, description)

decoder/             Core decoder (portable C; no ESP-IDF dependency in the math)
  lhdc_dec.c/.h         Public API, top-level frame decode, per-channel pipeline, gain/level
  lhdc_dec_internal.h   Decoder struct, size limits, workspace layout
  lhdc_entropy_dec.c/.h FAC range decoder + Rice quotient decode
  lhdc_sns_synth.c/.h   SNS (spectral noise shaping) scalefactor synthesis
  lhdc_imdct.c/.h       Fast inverse MDCT (FFT-based, sizes 480/960/1920)
  lhdc_tables.c/.h      Band configs, synthesis windows, bitrate tables
  lhdc_bit_reader.c/.h  64-bit-cache MSB-first bit reader
  lhdc_diag_config.c/.h Optional runtime diagnostics (off by default)
  imdct_const_tables.inc

a2dp_integration/    Bluedroid A2DP-sink glue (ESP-IDF)
  a2dp_vendor_lhdcv5.c/.h            Codec capability / negotiation
  a2dp_vendor_lhdcv5_decoder.c/.h    Frame reassembly + workspace mgmt + decode call
  a2dp_vendor_lhdcv5_constants.h     Vendor/codec IDs, sampling-freq bits

test/                Host verification tools (build on PC/Android, not ESP32)
  lhdc_roundtrip.c   Encode with real encoder -> decode with this decoder -> compare PCM
  fac_roundtrip.c    Standalone FAC range-coder round-trip
  test_roundtrip.py, roundtrip_pr.py  PCM analysis (THD, sideband, correlation)

docs/                Technical notes on specific fixes (windowing, channel selector, etc.)
```

The minimum you need to decode is everything in `decoder/`. `a2dp_integration/` is only
needed when wiring the decoder into a Bluedroid A2DP sink.

---

## Add to your ESP-IDF project

This repo **is** a self-contained ESP-IDF component. Drop it into your project's
`components/` directory — copy it, or add it as a git submodule:

```sh
cd your_project
git submodule add https://github.com/WillyBilly06/LHDC-V5-Decoder.git \
    components/lhdcv5_decoder
```

Then list it in the `REQUIRES` of whichever component calls the decoder — e.g. in
`main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES lhdcv5_decoder)
```

That's it — `#include "lhdc_dec.h"` and the decoder API is available (see
[Quick start](#quick-start)). The component compiles only the portable decoder in
`decoder/`, pulls in just `log` / `esp_common` / `esp_timer`, and builds for the
classic dual-core ESP32 (and any variant with enough internal RAM). The
`a2dp_integration/` and `test/` folders are shipped for reference but are **not**
part of the component build.

> **Decoding audio from a phone (A2DP sink)** is a further step: it requires
> patching Bluedroid's vendor-codec path with the files in `a2dp_integration/`.
> See [Integrating into an ESP-IDF A2DP sink](#integrating-into-an-esp-idf-a2dp-sink).

---

## Quick start

Decoding a raw LHDC V5 frame stream with the bare decoder API:

```c
#include "lhdc_dec.h"
#include <stdlib.h>

/* 1. Size and allocate the workspace for the negotiated rate (internal RAM on ESP32). */
size_t ws_size = lhdc_dec_get_workspace_size(48000 /* Hz */, 5 /* ms */);
void  *ws      = malloc(ws_size);

/* 2. Configure and initialize. config may be NULL to auto-detect from the bitstream. */
lhdc_dec_config_t cfg = {
    .sample_rate     = LHDC_DEC_SR_48000,
    .bit_depth       = LHDC_DEC_BITDEPTH_24,
    .frame_duration  = LHDC_DEC_FRAME_5MS,
    .channels        = 2,
    .max_frame_bytes = 1024,
    .lossless_enable = 0,
};
lhdc_decoder_t *dec = lhdc_dec_init(ws, &cfg);

/* 3. Decode. out_pcm holds samples_per_channel * channels samples.
 *    samples_per_channel = mdct_size/2: 240 @48k, 480 @96k, 960 @192k.
 *    24-bit output is written as 32-bit little-endian containers. */
int32_t pcm[960 * 2];                 /* big enough for the 192k case */
size_t   consumed  = 0;
uint32_t generated = 0;
lhdc_dec_ret_t r = lhdc_dec_decode_frame(dec, frame, frame_len,
                                         pcm, 960, &consumed, &generated, NULL);
if (r == LHDC_DEC_OK) {
    /* `generated` samples per channel are now interleaved in `pcm`. */
}

/* 4. The workspace owns all decoder state; free it when done. */
free(ws);
```

A real A2DP payload usually contains several frames back-to-back; call
`lhdc_dec_decode_frame` in a loop, advancing your input pointer by `consumed` each time
until the payload is exhausted.

---

## Decoder API reference

All functions are declared in `decoder/lhdc_dec.h` and are C-linkage.

### `size_t lhdc_dec_get_workspace_size(uint32_t sample_rate, uint8_t frame_duration)`
Returns the number of bytes the caller must allocate for the decoder workspace at the given
rate. The work buffers are rate-sized (driven by the MDCT size), so 48 kHz needs far less
than 96/192 kHz. `frame_duration` is in milliseconds (use `5`).

### `lhdc_decoder_t *lhdc_dec_init(void *workspace, const lhdc_dec_config_t *config)`
Initializes a decoder instance inside the caller-provided `workspace` (sized via
`lhdc_dec_get_workspace_size`). `config` may be `NULL` to auto-detect the format from the
first decoded frame. Returns an opaque handle, or `NULL` on failure. The handle lives
entirely inside `workspace`; there is no separate free function — release the workspace.

### `lhdc_dec_ret_t lhdc_dec_decode_frame(...)`
```c
lhdc_dec_ret_t lhdc_dec_decode_frame(
    lhdc_decoder_t *dec,
    const uint8_t  *in_data,    /* encoded input */
    size_t          in_bytes,   /* bytes available at in_data */
    void           *out_pcm,    /* interleaved PCM out, native endian */
    uint32_t        out_samples,/* capacity of out_pcm in samples-per-channel */
    size_t         *consumed,   /* [out] input bytes consumed by this frame */
    uint32_t       *generated,  /* [out] samples-per-channel written */
    lhdc_dec_frame_info_t *info /* [out] optional per-frame info, may be NULL */);
```
Decodes exactly one frame. Returns `LHDC_DEC_OK` on success. On error it returns one of the
[return codes](#return-codes) and leaves `out_pcm` untouched.

### `void lhdc_dec_flush(lhdc_decoder_t *dec)`
Clears the overlap-add history (use on a seek/discontinuity so the next frame doesn't smear
the previous content).

### `void lhdc_dec_reset(lhdc_decoder_t *dec)`
Resets the decoder to its just-initialized state.

### `lhdc_dec_ret_t lhdc_dec_get_config(lhdc_decoder_t *dec, lhdc_dec_config_t *config)`
Fills `config` with the active configuration (useful after auto-detect).

### `const char *lhdc_dec_strerror(lhdc_dec_ret_t ret)`
Returns a human-readable string for a return code.

---

## Data types

### Configuration — `lhdc_dec_config_t`
```c
typedef struct {
    lhdc_dec_sample_rate_t    sample_rate;     /* 44100/48000/96000/192000 */
    lhdc_dec_bitdepth_t       bit_depth;       /* 16 or 24 (32 = container width) */
    lhdc_dec_frame_duration_t frame_duration;  /* 5 ms */
    uint8_t                   channels;        /* 1 or 2 */
    uint32_t                  max_frame_bytes; /* largest encoded frame you'll feed */
    uint8_t                   lossless_enable; /* reserved; 0 */
} lhdc_dec_config_t;
```

### Frame info — `lhdc_dec_frame_info_t`
Populated by `lhdc_dec_decode_frame` when `info != NULL`: `frame_index`,
`encoded_frame_bytes`, `samples_per_channel`, `channels`, `sample_rate`, `bit_depth`,
`frame_duration_ms`, `version`, `ext_func_flags`, `target_bitrate`.

### Return codes
| Code | Value | Meaning |
|------|:-----:|---------|
| `LHDC_DEC_OK` | 0 | success |
| `LHDC_DEC_ERROR` | -1 | generic failure |
| `LHDC_DEC_INVALID_PARAM` | -2 | bad argument |
| `LHDC_DEC_INVALID_HANDLE` | -3 | bad/`NULL` handle |
| `LHDC_DEC_NOT_INITIALIZED` | -4 | decode before init |
| `LHDC_DEC_BUF_NOT_ENOUGH` | -5 | `out_pcm` too small |
| `LHDC_DEC_BITSTREAM_ERROR` | -6 | malformed frame |
| `LHDC_DEC_UNSUPPORTED_VERSION` | -7 | unknown stream version |
| `LHDC_DEC_UNSUPPORTED_SR` | -8 | unsupported sample rate |
| `LHDC_DEC_UNSUPPORTED_FORMAT` | -9 | unsupported format |
| `LHDC_DEC_NEED_MORE_DATA` | -10 | partial frame |

---

## Memory model

- The decoder keeps **all** state (struct + every per-frame buffer) inside the single
  `workspace` block you pass to `lhdc_dec_init`. There is no hidden heap allocation in the
  steady-state decode path.
- Workspace size is rate-driven (grow-only if you reuse one decoder across rate switches):

  | Rate | MDCT | Approx. workspace |
  |------|:----:|:-----------------:|
  | 44.1 / 48 kHz | 480 | ~10 KB |
  | 96 kHz | 960 | ~18 KB |
  | 192 kHz | 1920 | ~32.5 KB |

- On the ESP32, allocate the workspace in **internal** RAM (`MALLOC_CAP_INTERNAL`). The hot
  buffers are read/written per sample; placing them in PSRAM would slow decode badly.
- The fast IMDCT twiddle tables are allocated separately (lazily, per active size) and must
  also stay in internal RAM for the same reason. They are freed automatically when you
  reconfigure to a different rate.

---

## How the decode pipeline works

A 5 ms LHDC V5 frame carries two independently-coded channels. Each channel is decoded as:

1. **Header descramble** — each channel's 8-byte header is descrambled independently.
2. **SNS side info** — a 4-bit mode + per-band direction bits reconstruct per-band scale
   factors (adsq DPCM).
3. **Spectral data** — two ternary streams (an LSB/marker plane and the Rice quotient codes)
   are **FAC range coded** with sliding-window adaptive models, sharing one byte stream.
4. **Mantissa + sign plane** — a causal bit-plane reconstructs full coefficient magnitudes
   from the Rice quotients (the shift is predicted per coefficient), plus a sign bit per
   non-zero coefficient.
5. **Inverse quantization** — `coeff * 2^gain_exponent` using the frame's global gain step.
6. **SNS synthesis** — undo the per-band spectral shaping.
7. **LD-IMDCT** — fast inverse MDCT (size = 2 x samples-per-channel: 480/960/1920),
   implemented as a length-N/4 complex FFT with pre/post twiddle and a symmetric unfold.
8. **Windowed overlap-add** — low-overlap synthesis window with 50% TDAC overlap.

The two channels are then interleaved to stereo PCM.

### IMDCT internals
The fast IMDCT factors the N/4-point FFT as a four-step transform: a radix-2 stage
(8/16/32-point for N = 480/960/1920) followed by a 15-point stage implemented as a 3x5
Cooley-Tukey DFT. A one-shot self-test at init validates each fast path against a direct
cosine-formula reference; if it fails (e.g. out-of-memory for the twiddle tables) the
decoder falls back to the slow reference IMDCT rather than producing wrong output. Watch
the init log line `IMDCT-<N> self-test: fast=ENABLED maxdiff=...` to confirm the fast path.

---

## Integrating into an ESP-IDF A2DP sink

The `a2dp_integration/` layer plugs the decoder into Bluedroid's vendor-codec sink path.

1. **Place the sources.** Add `decoder/` and `a2dp_integration/` to the Bluedroid tree
   (`components/bt/host/bluedroid/...`) or as an IDF component, and add them to the build.
2. **Register the codec.** Use the vendor ID / codec ID and the sampling-frequency bit
   definitions in `a2dp_vendor_lhdcv5_constants.h`. Advertise the rates you support in the
   sink capability table (`a2dp_vendor_lhdcv5.c`).
3. **Wire the decoder callbacks** (`a2dp_vendor_lhdcv5_decoder.h`):
   ```c
   bool    a2dp_lhdcv5_decoder_init(decoded_data_callback_t cb);   /* cb receives PCM */
   void    a2dp_lhdcv5_decoder_configure(const uint8_t *codec_info);/* on SET_CONFIG */
   ssize_t a2dp_lhdcv5_decoder_decode_packet_header(BT_HDR *p);     /* strip RTP/frag hdr */
   bool    a2dp_lhdcv5_decoder_decode_packet(BT_HDR *p, uint8_t *out, size_t out_len);
   void    a2dp_lhdcv5_decoder_cleanup(void);
   ```
   `configure` parses the negotiated CIE, sizes/allocates the workspace, and calls
   `lhdc_dec_init`. `decode_packet` reassembles fragments and calls
   `lhdc_dec_decode_frame`, delivering PCM through your `decoded_data_callback_t`.
4. **Output container.** 24-bit is delivered as interleaved 32-bit little-endian samples;
   feed your I2S path accordingly.

The decode runs on Bluedroid's A2DP sink task. Pin that task and your audio render task to
**different cores** so decode doesn't starve the I2S writer.

---

## 192 kHz / PSRAM build

192 kHz is twice the per-second work of 96 kHz, and hits two limits on a stock ESP32:

- **RAM:** the ~32.5 KB workspace plus IMDCT tables don't fit alongside the Bluetooth stack
  in internal SRAM on a WROOM. The fix is PSRAM (WROVER): route the Bluedroid **host** and
  large ring buffers to PSRAM, freeing internal SRAM for the decoder's hot buffers.
- **CPU:** a single core is at the edge for 192 kHz, so high bitrates outrun real time on
  one core. The path to smooth 192 kHz is a **dual-core stereo split** (decode L and R on
  separate cores), which only becomes feasible once PSRAM has freed the internal SRAM for
  the second channel's buffers. Treat 192 kHz as experimental until that split is in place.

PSRAM `sdkconfig` settings used (ESP32-WROVER, chip rev >= 3):
```
CONFIG_SPIRAM=y
CONFIG_SPIRAM_MODE_QUAD=y
CONFIG_SPIRAM_TYPE_AUTO=y
CONFIG_SPIRAM_SPEED_40M=y
CONFIG_SPIRAM_BOOT_INIT=y
CONFIG_SPIRAM_USE_MALLOC=y
CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST=y   # move Bluedroid host to PSRAM
CONFIG_ESP32_REV_MIN_3=y                   # rev>=3: drops the PSRAM cache workaround (frees IRAM)
```
`CONFIG_ESP32_REV_MIN_3` is important: on rev < 3 the PSRAM cache workaround forces a large
block of libc into IRAM and overflows it. Setting the minimum revision to 3 (valid on a
v3.x chip) removes the workaround and reclaims that IRAM. 44.1/48/96 kHz need none of this.

---

## Testing

`test/lhdc_roundtrip.c` is the primary correctness harness. It encodes a known signal with
the **real** LHDC encoder, decodes it with this decoder, and writes both PCM streams for
comparison. Because LHDC is lossy, the test validates spectral fidelity (correct frequency,
low THD, no frame-rate sidebands), not bit-exact PCM.

It requires the proprietary `liblhdcv5.so` encoder at runtime (provide your own; not
included). Build it with the Android NDK against the decoder sources:

```sh
aarch64-linux-android21-clang test/lhdc_roundtrip.c \
    decoder/lhdc_dec.c decoder/lhdc_entropy_dec.c decoder/lhdc_sns_synth.c \
    decoder/lhdc_imdct.c decoder/lhdc_tables.c decoder/lhdc_bit_reader.c \
    decoder/lhdc_diag_config.c \
    -DLHDC_HOST_BUILD -Idecoder -ldl -lm -o lhdc_roundtrip
```

Run on a device that has `liblhdcv5.so`:
```sh
# generate a test tone as raw interleaved PCM, then:
LHDC_SR=96000 LHDC_BPS=24 LHDC_CH=2 LHDC_BR=900 \
    ./lhdc_roundtrip ./liblhdcv5.so in.pcm out.pcm
```
Environment knobs: `LHDC_SR` (sample rate), `LHDC_BPS` (16/24), `LHDC_CH` (1/2),
`LHDC_BR` (target kbps).

Analyze the result with the Python scripts (NumPy):
```sh
python test/test_roundtrip.py in.pcm out.pcm     # THD+N, RMS/peak error, correlation
python test/roundtrip_pr.py  in.pcm out.pcm      # sideband energy around fixed tones
```

`test/fac_roundtrip.c` is a smaller harness that round-trips just the FAC range coder.

---

## Verification results

Validated by phone-encoder round-trip (real `liblhdcv5.so`) and on-device playback:

- **Bit-exact decode** confirmed on the host round-trip across all configurations,
  including 96 kHz and 192 kHz (decoded spectrum matches the encoder input within lossy
  tolerance, no added leakage).
- **Scale-factor (SNS) and gain reconstruction are now bit-exact against the reference.**
  This corrected an earlier low-frequency error that showed up as elevated THD / "watery"
  distortion on sub-~50 Hz tones and slow bass sweeps at 96/192 kHz, plus a level-scaling
  bug that clipped 192 kHz. Every sample rate and bit depth now reconstructs down to the
  codec's clean quantization floor.
- **THD (tone in -> tone out):** ~-64 dB across 44.1/48/96/192 kHz and both bit depths
  (uniform at the lossy quantization floor, including the low bass region that used to
  distort).
- **IMDCT self-tests** (480 / 960 / 1920) pass against the cosine-formula reference.
- **On-device (ESP32):** 44.1 / 48 / 96 kHz play clean and glitch-free in real time at all
  bitrates (256 k -> 900 k + Auto).

| Sample rate | Bit depth | 256k | 400k | 500k | 900k | Auto | Result |
|-------------|:---------:|:----:|:----:|:----:|:----:|:----:|--------|
| 44.1 kHz | 16/24 | ok | ok | ok | ok | ok | clean, real-time |
| 48 kHz   | 16/24 | ok | ok | ok | ok | ok | clean, real-time |
| 96 kHz   | 24    | ok | ok | ok | ok | ok | clean, real-time |
| 192 kHz  | 24    | decodes | decodes | decodes | decodes | decodes | correct PCM; needs PSRAM + dual-core for real-time |

---

## Performance & Xtensa tuning

At 96 kHz on the ESP32 @ 240 MHz, decode is roughly 1.2 ms per channel per 5 ms frame,
split approximately: entropy ~38%, IMDCT ~a quarter, and the remaining
mantissa/SNS/overlap work the rest — comfortably within budget on one core.

Xtensa LX6-specific optimizations applied:
- Hot functions (`fac_decode`, Rice/mantissa, IMDCT, overlap-add, inverse-quant) are placed
  in IRAM to avoid flash-cache stalls under Bluetooth interrupts.
- Single-precision FPU throughout; no `double`, no transcendental calls in per-sample loops
  (the gain step uses one `exp2f` per channel; SNS gains are table lookups).
- The FAC range coder avoids the per-symbol integer divide using the identity
  `floor(code/r) >= c  <=>  code >= r*c` (Xtensa has no fast hardware divide).
- The IMDCT is FFT-based (no O(N^2) transform); the 15-point DFT stage uses a 3x5
  Cooley-Tukey factorization, and bit-width counts use the `NSAU` (count-leading-zeros)
  instruction.
- Twiddle tables live in DRAM/IRAM (never flash) because they are read thousands of times
  per frame.

---

## Diagnostics

- `extern volatile int g_lhdc_trace;` (in `lhdc_dec.h`) — set to `1` to log the first
  decoded frame's leading-section values; the decoder clears it after one frame.
- `lhdc_diag_config.h` exposes optional runtime switches (e.g. forcing the reference IMDCT,
  SNS direction) that default to off and are intended for bring-up only.

---

## Troubleshooting

- **`decode_packet null guard (dec=0x0)` / no audio:** the workspace allocation failed —
  the heap is too fragmented for the (contiguous) workspace block. At 192 kHz this means
  you need PSRAM and should allocate the workspace early/once. See
  [192 kHz / PSRAM build](#192-khz--psram-build).
- **Slowed-down / "slow-motion" audio at high bitrate:** CPU starvation — the decoder is
  correct but isn't keeping up on a single core. This is the 192 kHz dual-core case.
- **`IMDCT-<N> self-test: fast=DISABLED`:** the fast IMDCT tables couldn't allocate; the
  decoder fell back to the slow reference path (correct but far slower). Free internal RAM.
- **Clicks/pops on codec switch:** flush the decoder (`lhdc_dec_flush`) and ramp the output
  on a reconfigure so the IMDCT overlap history doesn't smear across the discontinuity.

---

## Limitations

- 192 kHz is experimental: correct but not real-time on a single core (see above).
- Frame duration is 5 ms (LHDC V5's stream duration); 7.5/10 ms enums exist but the live
  path is the 5 ms framing.
- `lossless_enable` is reserved.

---

## License

This decoder implementation is released under the **Apache License 2.0**. Note that this
does **not** grant any rights to the LHDC codec itself, which is the property of Savitech;
obtaining LHDC licensing for your use case is your responsibility.
