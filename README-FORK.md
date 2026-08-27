# esp-idf-v6.1-codecs

A fork of **ESP-IDF v6.1** carrying extra A2DP **sink** codecs for Bluedroid,
including a from-scratch **LHDC V5** decoder that plays 192 kHz / 24-bit in real
time on a classic ESP32.

This is a **self-contained snapshot**: every git submodule Espressif normally
pulls in (BT controller libs, PHY, WiFi, mbedTLS, NimBLE, …) is vendored here as
ordinary files, so a plain `git clone` gives you a tree that builds. There is no
`.gitmodules` and `git submodule update` is neither needed nor meaningful.

Upstream history is not included — this is a snapshot, not a rebase of
Espressif's tree. Upstream lives at <https://github.com/espressif/esp-idf>
(Apache-2.0, as is everything inherited from it here).

## What is different from stock v6.1

Everything of interest is under `components/bt/host/bluedroid/`:

| path | what |
|---|---|
| `external/lhdc-v5/` | the LHDC V5 decoder (also published standalone at [WillyBilly06/LHDC-V5-Decoder](https://github.com/WillyBilly06/LHDC-V5-Decoder)) |
| `external/libaac-lc/` | Helix-based AAC-LC decoder |
| `stack/a2dp/a2dp_vendor_lhdcv5*` | LHDC V5 codec info / negotiation / decoder glue |
| `stack/a2dp/a2dp_aac_decoder_custom.c` | AAC sink decoder wiring |
| `btc/profile/std/a2dp/btc_a2dp_sink.c` | sink task sizing, RxQ limits, decoder reset ordering |

A worked sink application using all of it lives in
`examples/bluetooth/bluedroid/classic_bt/` (see the `a2dp_sink_int_codec_utils`
util, which contains the output-ring sizing, backpressure and I2S source-clock
tracking that hi-res playback needs).

## Building

Standard IDF flow — no submodule step:

```
. ./export.sh          # or export.ps1 on Windows
idf.py set-target esp32
idf.py build
```

`esp32s31` is a preview target and needs `idf.py --preview set-target esp32s31`.

## Licence

Apache-2.0, inherited from ESP-IDF. See `LICENSE`.
