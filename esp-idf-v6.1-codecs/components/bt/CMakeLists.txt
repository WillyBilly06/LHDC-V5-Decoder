idf_build_get_property(target IDF_TARGET)

if(${target} STREQUAL "linux")
    return() # This component is not supported by the POSIX/Linux simulator
endif()

if(CONFIG_IDF_TARGET_ESP32S3)
    set(TARGET_SRC_NAME "esp32c3")
elseif(CONFIG_IDF_TARGET_ESP32C61)
    set(TARGET_SRC_NAME "esp32c6")
elseif(CONFIG_IDF_TARGET_ESP32H21)
    set(TARGET_SRC_NAME "esp32h2")
else()
    set(TARGET_SRC_NAME "${idf_target}")
endif()

set(srcs "")
set(include_dirs "")
set(priv_include_dirs "")
set(ldscripts "linker_common.lf")

# API headers that are used in the docs are also compiled
# even if CONFIG_BT_ENABLED=n as long as CONFIG_IDF_DOC_BUILD=y
if(CONFIG_IDF_DOC_BUILD OR CONFIG_BT_ENABLED)
    add_subdirectory(common)
    list(APPEND srcs ${bt_common_srcs})
    list(APPEND include_dirs ${bt_common_include_dirs})
    list(APPEND priv_include_dirs ${bt_common_priv_include_dirs})

    # Controller
    add_subdirectory(controller)
    list(APPEND srcs ${bt_ctrl_srcs})
    list(APPEND include_dirs ${bt_ctrl_include_dirs})
    set(ldscripts ${bt_ctrl_ldscripts})

    # Controller porting or porting_btdm
    if(CONFIG_SOC_ESP_NIMBLE_CONTROLLER)
        if(CONFIG_BT_DUAL_MODE_ARCH)
            add_subdirectory(porting_btdm)
        else()
            add_subdirectory(porting)
        endif()
        list(APPEND srcs ${porting_srcs})
        list(APPEND include_dirs ${porting_include_dirs})
        list(APPEND priv_include_dirs ${porting_priv_include_dirs})
    endif()

    # Host Bluedroid
    add_subdirectory(host/bluedroid)
    list(APPEND srcs ${bluedroid_host_srcs})
    list(APPEND include_dirs ${bluedroid_host_include_dirs})
    list(APPEND priv_include_dirs ${bluedroid_host_priv_include_dirs})

    # NimBLE Host
    add_subdirectory(host/nimble)
    list(APPEND srcs ${nimble_host_srcs})
    list(APPEND include_dirs ${nimble_host_include_dirs})

    # BLE Mesh
    add_subdirectory(esp_ble_mesh)
    list(APPEND srcs ${ble_mesh_srcs})
    list(APPEND include_dirs ${ble_mesh_include_dirs})

    # BLE ISO (host shim + ISO transport, reusable by BLE Audio / HID-over-ISO / etc.)
    add_subdirectory(esp_ble_iso)
    list(APPEND srcs ${ble_iso_srcs})
    list(APPEND include_dirs ${ble_iso_include_dirs})

    # BLE Audio
    add_subdirectory(esp_ble_audio)
    list(APPEND srcs ${ble_audio_srcs})
    list(APPEND include_dirs ${ble_audio_include_dirs})

    # When log compression is enabled, selected logs are replaced
    # by auto-generated macros that emit pre-encoded data.
    # This eliminates the original format strings, reducing firmware size and
    # removing runtime formatting overhead, so logs are produced faster and
    # with less system impact.
    replace_log_compression_srcs()
endif()

set(bt_priv_requires
    nvs_flash
    soc
    esp_pm
    esp_phy
    esp_coex
    mbedtls
    esp_driver_uart
    vfs
    esp_ringbuf
    esp_gdbstub
    esp_security
    esp_event
)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       PRIV_INCLUDE_DIRS "${priv_include_dirs}"
                       REQUIRES esp_timer esp_wifi esp_driver_spi esp_driver_gpio
                       PRIV_REQUIRES "${bt_priv_requires}"
                       LDFRAGMENTS "${ldscripts}")

idf_component_get_property(bt_component_type bt COMPONENT_TYPE)
if(bt_component_type STREQUAL "LIBRARY")
    idf_component_optional_requires(PRIVATE ble_insights)
else()
    idf_component_optional_requires(INTERFACE ble_insights)
endif()

if(CONFIG_BT_ENABLED)
    target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-implicit-fallthrough -Wno-unused-const-variable)
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
    # GCC 15 (riscv32-esp-elf) flags exact-fit char[] codec-capability string inits;
    # harmless for the ported A2DP codec tables, so don't treat it as an error.
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=unterminated-string-initialization")
    # Third-party vendor codec libraries (LDAC/Opus/LHDC) trip several pedantic
    # warnings under -Werror; they are benign, so demote them for this component.
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=type-limits")
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=shift-negative-value")
    # At -O2, GCC's stringop analysis flags the stack's BCM_STRNCPY_S (strncpy of
    # BD_NAME_LEN bytes) as possibly-truncated. It's a false positive on the fixed
    # bd_name field and only appears above -Og; demote it for this component.
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=stringop-truncation")
    # Opus/CELT trip -Wmaybe-uninitialized under -O2's deeper flow analysis
    # (well-known false positives in the vendored codec); demote to warnings.
    target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=maybe-uninitialized")
    # Build the LHDC-V5 decoder hot path at -O3 (the component default is -O2).
    # -O3 unrolls/vectorizes the IMDCT butterfly + entropy/rice inner loops harder;
    # results stay BIT-EXACT (no -ffast-math). This gives the classic ESP32 the last
    # few % it needs to sustain 192k/1000 kbps (profiler: entropy peaks ~1240 us vs
    # the 2500 us/channel budget). The S31 is unaffected (already keeps up).
    if(CONFIG_BT_A2DP_LHDCV5_DECODER)
        set_source_files_properties(
            "${CMAKE_CURRENT_SOURCE_DIR}/host/bluedroid/external/lhdc-v5/lhdc_dec.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/host/bluedroid/external/lhdc-v5/lhdc_entropy_dec.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/host/bluedroid/external/lhdc-v5/lhdc_imdct.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/host/bluedroid/external/lhdc-v5/lhdc_sns_synth.c"
            "${CMAKE_CURRENT_SOURCE_DIR}/host/bluedroid/external/lhdc-v5/lhdc_bit_reader.c"
            TARGET_DIRECTORY ${COMPONENT_LIB}
            PROPERTIES COMPILE_OPTIONS "-O3")
    endif()
    # Audited 2026-07 with -Wdouble-promotion: the LHDC decoder compute path is
    # 100% single-precision (no double types, no double libm, no double literals,
    # no per-sample division/transcendentals -- those are table-precomputed). The
    # only float->double conversions are the unavoidable C variadic promotion in
    # diagnostic ESP_LOGI/printf (%f) lines (debug traces + init self-tests), not
    # in any audio hot loop. Re-add "-Wdouble-promotion" above to re-audit.
    # Opus is built with its bundled config.h (defines OPUS_BUILD/VAR_ARRAYS/FIXED_POINT/HAVE_LRINTF)
    if(CONFIG_BT_A2DP_OPUS_DECODER)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE "HAVE_CONFIG_H")
    endif()
    # Helix AAC's assembly.h uses the generic portable-C code path when ESP32 is
    # defined (a platform selector, not target-specific); valid for the S31 RISC-V core too.
    if(CONFIG_BT_A2DP_AAC_DECODER)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE "ESP32")
    endif()

    register_bt_ctrl_libs()
    set_bluedroid_host_compile_flags()
    set_nimble_host_compile_flags()
    register_ble_mesh_libs()
    register_ble_audio_libs()

    register_log_compression_dependency()
    wrap_uart_log_tx_interface()
endif()
