# WROVER_A2DP_Sink — hi-res Bluetooth A2DP receiver → I2S

The application this decoder was developed and measured against. A phone or PC
connects over Bluetooth Classic and streams music; the board decodes it and
outputs PCM to an external I2S DAC.

It is the reference for the whole stack: **LHDC V5 at 192 kHz / 24-bit plays in
real time on a plain 240 MHz classic ESP32**, single core, with no PSRAM.

> **The folder name is historical.** Development started on an ESP32-WROVER, but
> the board it is now tuned and measured on is an **ESP32-WROOM (ESP32-D0WD-V3
> rev 3.1, 4 MB flash, no PSRAM)**. The name was kept so it matches the
> measurements quoted throughout this repository. It also builds for `esp32s31`.

## Codecs

All decode in software on the chip. Enabled in `sdkconfig.defaults`:

| codec | notes |
|---|---|
| SBC | stock |
| AAC-LC | Helix-based (`components/bt/host/bluedroid/external/libaac-lc`) |
| LDAC | up to 96 kHz |
| aptX / aptX HD | |
| **LHDC V5** | 44.1 / 48 / 96 / **192 kHz**, 16 and 24-bit — this repository |
| Opus | Google and PipeWire vendor IDs |
| LC3plus | |

## Hardware

| | |
|---|---|
| Chip | ESP32-D0WD-V3 (WROOM), 4 MB flash, PSRAM absent and disabled |
| CPU | 240 MHz (`CONFIG_ESP32_DEFAULT_CPU_FREQ_240`) — the default 160 MHz is not enough for 192 kHz |
| I2S | BCLK **GPIO27**, WS/LRCK **GPIO25**, DOUT **GPIO26**, no MCLK |
| I2S format | always 32-bit stereo Philips; the decoder's 16/24-bit PCM is up-converted |
| Console | **921600 baud** — see below |

Pins are `CONFIG_EXAMPLE_I2S_*` in `sdkconfig.defaults.esp32`; change them there.

## Build and flash

Use the ESP-IDF fork bundled in this repository (`../esp-idf-v6.1-codecs`). It
already contains the codecs and needs no submodule step.

```
cd ../esp-idf-v6.1-codecs && . ./export.sh     # export.ps1 on Windows
cd ../WROVER_A2DP_Sink
idf.py set-target esp32
idf.py -p <PORT> flash monitor
```

`sdkconfig` is deliberately **not** committed — it is generated from
`sdkconfig.defaults` + `sdkconfig.defaults.esp32`, and a stale checked-in copy
silently overrides them.

### Console baud

The console runs at **921600, not 115200**. `ESP_LOGx` blocks the calling task
until the bytes leave the UART; at 115200 a single ~120-character line costs
~10 ms, which is *two entire* 5 ms LHDC frame budgets, and a periodic
three-line status print was enough to audibly stutter 192 kHz playback on its
own. Set your terminal accordingly (`idf.py monitor` reads it from sdkconfig).

Note `CONFIG_ESP_CONSOLE_UART_BAUDRATE` is only settable alongside
`CONFIG_ESP_CONSOLE_UART_CUSTOM=y`; with the default console channel kconfig
silently reverts it to 115200. Both are set in `sdkconfig.defaults`. Verify in
`build/config/sdkconfig.h`, not in `sdkconfig`.

## What to look for in the log

```
LHDC-RT: frames=1001 avg=2945 us max=4699 us budget=5000 us (93% peak) over=0
```
Per-frame decode cost against the 5 ms real-time budget. `over=` is the number
of frames that missed it — it should be **0**.

```
output ring budget: 8-bit DRAM free 70 KB - 24 KB BT reserve -> target 44 KB
```
The PCM ring is sized from *measured* free byte-addressable DRAM after the
decoder has claimed its rate-sized buffers, so 192 kHz automatically gets a
smaller ring than 48 kHz.

```
underflow: input 97% of realtime over 2096 ms (3133440 B in, 92160 B dropped-full)
```
Only on underflow. `input` below 100% means starvation upstream; a non-zero
`dropped-full` means the ring overflowed and audio was discarded.

```
clock track: ring 28% -> mclk -512 Hz (now 49151488 Hz, -10 ppm vs source)
```
The source and this board run off different crystals (measured ~127 ppm apart),
which drains any fixed buffer at a constant rate — a brief stutter every few
minutes, forever. The I2S rate is tuned to follow the source via
`i2s_channel_tune_rate()`. These lines should appear a handful of times after a
stream starts and then stop.

```
HEAP: internal free: ... || 8-bit DRAM free: 22084 B | largest: 18432 B
```
`8-bit DRAM free` is the one that matters on a classic ESP32: it is the pool the
Bluetooth media allocator, the decoder work buffers and the output ring all
share. If it approaches zero you get `BT_OSI: malloc failed` and the stream dies.

## Licence

Apache-2.0 / CC0, following the ESP-IDF example this is derived from.
