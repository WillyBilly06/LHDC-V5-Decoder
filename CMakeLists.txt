# ESP-IDF component: LHDC V5 software decoder.
#
# Drop this repo into your project's `components/` directory (copy or
# `git submodule add`), and the decoder API in `decoder/lhdc_dec.h` becomes
# available to any component that lists `lhdcv5_decoder` in its REQUIRES.
#
# Only the portable decoder in `decoder/` is compiled here. The files under
# `a2dp_integration/` are reference glue for a Bluedroid A2DP sink (they patch
# the BT stack, so they are NOT built as part of this component — see README),
# and `test/` is host-only tooling.
idf_component_register(
    SRCS
        "decoder/lhdc_bit_reader.c"
        "decoder/lhdc_dec.c"
        "decoder/lhdc_diag_config.c"
        "decoder/lhdc_entropy_dec.c"
        "decoder/lhdc_imdct.c"
        "decoder/lhdc_sns_synth.c"
        "decoder/lhdc_tables.c"
    INCLUDE_DIRS
        "decoder"
    PRIV_REQUIRES
        log            # esp_log.h  (ESP_LOGx)
        esp_common     # esp_attr.h (IRAM_ATTR for the hot decode paths)
        esp_timer      # esp_timer.h
)
