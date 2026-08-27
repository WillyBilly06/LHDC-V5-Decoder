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
        heap           # esp_heap_caps.h (work buffers are allocated per rate)
        freertos       # only when LHDC_DEC_DUAL_CORE is enabled in lhdc_dec_slot.h
)

# Build the decode path at -O3 regardless of the project's global optimization
# level. Measured on a 240 MHz classic ESP32 at 192 kHz/24-bit: the IMDCT alone
# went 795 us -> 626 us per channel and the whole frame 2545 us -> 2163 us,
# which is the difference between holding the 5 ms real-time budget and not.
# These seven files are pure DSP with no I/O, so -O3's aggressive inlining and
# loop transforms cost only flash, not correctness.
foreach(lhdc_src
        lhdc_bit_reader.c lhdc_dec.c lhdc_entropy_dec.c
        lhdc_imdct.c lhdc_sns_synth.c lhdc_tables.c)
    set_source_files_properties("decoder/${lhdc_src}"
        TARGET_DIRECTORY ${COMPONENT_LIB}
        PROPERTIES COMPILE_OPTIONS "-O3")
endforeach()
