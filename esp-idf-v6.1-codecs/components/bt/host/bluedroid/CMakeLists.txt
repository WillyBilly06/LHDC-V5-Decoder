# TODO: These warnings should be resolved in the Bluedroid code
function(set_bluedroid_host_compile_flags)
    if(NOT CONFIG_BT_BLUEDROID_ENABLED)
        return()
    endif()

    set_source_files_properties(
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bta/gatt/bta_gattc_act.c"
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bta/gatt/bta_gattc_cache.c"
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/btc/profile/std/gatt/btc_gatt_util.c"
        "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/btc/profile/std/gatt/btc_gatts.c"
        PROPERTIES COMPILE_FLAGS -Wno-address-of-packed-member
    )

    if(NOT CMAKE_BUILD_EARLY_EXPANSION)
        set(jump_table_opts "-fjump-tables")
        if(NOT (CMAKE_C_COMPILER_ID MATCHES "Clang") )
            set(jump_table_opts "${jump_table_opts} -ftree-switch-conversion")
        endif()
        set_source_files_properties("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/bta/hf_ag/bta_ag_cmd.c"
                                    "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/btc/profile/std/gap/btc_gap_ble.c"
                                    PROPERTIES COMPILE_FLAGS "${jump_table_opts}")
    endif()

    if((CMAKE_C_COMPILER_ID MATCHES "GNU" AND CMAKE_C_COMPILER_VERSION VERSION_GREATER 15.0) OR
        CMAKE_C_COMPILER_ID MATCHES "Clang")
        set_source_files_properties("${CMAKE_CURRENT_FUNCTION_LIST_DIR}/device/controller.c"
                                    PROPERTIES COMPILE_FLAGS "-Wno-unterminated-string-initialization")
    endif()
endfunction()

set(bluedroid_host_srcs "" PARENT_SCOPE)
set(bluedroid_host_include_dirs "" PARENT_SCOPE)
set(bluedroid_host_priv_include_dirs "" PARENT_SCOPE)

# API headers that are used in the docs are also compiled
# even if CONFIG_BT_ENABLED=n as long as CONFIG_IDF_DOC_BUILD=y
if(CONFIG_IDF_DOC_BUILD)
    set(bluedroid_host_include_dirs
        "${CMAKE_CURRENT_LIST_DIR}/api/include/api"
        PARENT_SCOPE
    )
    return()
endif()

if(NOT CONFIG_BT_BLUEDROID_ENABLED)
    return()
endif()

list(APPEND bluedroid_host_srcs
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_a2dp_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_avrc_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_bluedroid_hci.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_bt_device.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_bt_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_gap_ble_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_gap_bt_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_gatt_common_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_gattc_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_gatts_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_hidd_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_hidh_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_hf_ag_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_hf_client_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_spp_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_sdp_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_l2cap_bt_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/api/esp_pbac_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/ar/bta_ar.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_aact.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_ca_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_ca_sm.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_cfg.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_ci.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_sbc.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_m24.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/av/bta_av_ssm.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_cfg.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_ci.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_pm.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_sco.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/dm/bta_dm_qos.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gatt_common.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_cache.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_ci.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gattc_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gatts_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gatts_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gatts_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gatts_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/gatt/bta_gatts_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hd/bta_hd_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hd/bta_hd_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hd/bta_hd_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hh/bta_hh_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hh/bta_hh_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hh/bta_hh_cfg.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hh/bta_hh_le.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hh/bta_hh_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hh/bta_hh_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/jv/bta_jv_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/jv/bta_jv_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/jv/bta_jv_cfg.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/jv/bta_jv_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_at.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_cfg.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_cmd.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_rfc.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_sco.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/bta_ag_sdp.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_at.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_cmd.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_rfc.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_sco.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/bta_hf_client_sdp.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/pba/bta_pba_client_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/pba/bta_pba_client_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/pba/bta_pba_client_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/pba/bta_pba_client_sdp.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sdp/bta_sdp.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sdp/bta_sdp_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sdp/bta_sdp_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sdp/bta_sdp_cfg.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sys/bta_sys_conn.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sys/bta_sys_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/bta/sys/utl.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_ble_storage.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_config.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_dev.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_dm.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_profile_queue.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_sec.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_sm.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_storage.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/core/btc_util.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/bta_av_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_control.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_sink.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_sink_ext_coedc.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_latm_raw.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_source.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_source_ext_codec.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_av.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/avrc/btc_avrc.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/avrc/bta_avrc_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hf_ag/bta_ag_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hf_ag/btc_hf_ag.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hf_client/btc_hf_client.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hf_client/bta_hf_client_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hid/btc_hd.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hid/btc_hh.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/hid/bta_hh_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gap/btc_gap_ble.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gap/btc_gap_bt.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gap/bta_gap_bt_co.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gatt/btc_gatt_common.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gatt/btc_gatt_util.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gatt/btc_gattc.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/gatt/btc_gatts.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/spp/btc_spp.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/sdp/btc_sdp.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/l2cap/btc_l2cap.c"
    "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/pba/btc_pba_client.c"
    "${CMAKE_CURRENT_LIST_DIR}/device/bdaddr.c"
    "${CMAKE_CURRENT_LIST_DIR}/device/controller.c"
    "${CMAKE_CURRENT_LIST_DIR}/device/interop.c"
    "${CMAKE_CURRENT_LIST_DIR}/hci/hci_audio.c"
    "${CMAKE_CURRENT_LIST_DIR}/hci/hci_hal_h4.c"
    "${CMAKE_CURRENT_LIST_DIR}/hci/hci_layer.c"
    "${CMAKE_CURRENT_LIST_DIR}/hci/hci_packet_factory.c"
    "${CMAKE_CURRENT_LIST_DIR}/hci/hci_packet_parser.c"
    "${CMAKE_CURRENT_LIST_DIR}/hci/packet_fragmenter.c"
    "${CMAKE_CURRENT_LIST_DIR}/main/bte_init.c"
    "${CMAKE_CURRENT_LIST_DIR}/main/bte_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_sbc.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_m24.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avct/avct_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avct/avct_ccb.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avct/avct_l2c.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avct/avct_lcb.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avct/avct_lcb_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_ad.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_ccb.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_ccb_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_l2c.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_msg.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_scb.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avdt/avdt_scb_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_bld_ct.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_bld_tg.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_opt.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_pars_ct.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_pars_tg.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_sdp.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/avrc/avrc_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/hid/hidd_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/hid/hidd_conn.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/hid/hidh_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/hid/hidh_conn.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_acl.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_addr.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_adv_filter.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_batchscan.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_bgconn.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_cont_energy.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_gap.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_5_gap.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_multi_adv.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_privacy.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_dev.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_devctl.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_inq.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_bredr_pwr_ctrl.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_pm.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_sco.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_sec.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btu/btu_hcif.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btu/btu_init.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/btu/btu_task.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gap/gap_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gap/gap_ble.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gap/gap_conn.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gap/gap_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/att_protocol.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_attr.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_auth.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_cl.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_db.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_sr.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_sr_hash.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/gatt/gatt_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/goep/goepc_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/goep/goepc_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/hcic/hciblecmds.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/hcic/hcicmds.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_ble.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_csm.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_fcr.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_link.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_ucd.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2c_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/l2cap_client.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/obex/obex_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/obex/obex_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/obex/obex_tl_l2cap.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/obex/obex_tl_rfcomm.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/port_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/port_rfc.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/port_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/rfc_l2cap_if.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/rfc_mx_fsm.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/rfc_port_fsm.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/rfc_port_if.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/rfc_ts_frames.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/rfc_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/sdp/sdp_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/sdp/sdp_db.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/sdp/sdp_discovery.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/sdp/sdp_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/sdp/sdp_server.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/sdp/sdp_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/aes.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/p_256_curvepara.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/p_256_ecc_pp.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/p_256_multprecision.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_act.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_api.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_br_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_cmac.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_keys.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_l2c.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_main.c"
    "${CMAKE_CURRENT_LIST_DIR}/stack/smp/smp_utils.c"
    "${CMAKE_CURRENT_LIST_DIR}/config/stack_config.c"

    # TODO: Add this file in the blufi cmake file
    "${CMAKE_CURRENT_LIST_DIR}/../../common/btc/profile/esp/blufi/bluedroid_host/esp_blufi.c"
)

if(CONFIG_BT_BLE_FEAT_ISO_EN)
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_iso.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/iso/btc_iso_ble.c"
        "${CMAKE_CURRENT_LIST_DIR}/api/esp_ble_iso_api.c"
        "${CMAKE_CURRENT_LIST_DIR}/hci/ble_hci_iso.c"
    )
endif()

if(CONFIG_BT_BLE_FEAT_CTE_EN)
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/stack/btm/btm_ble_cte.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/cte/btc_ble_cte.c"
        "${CMAKE_CURRENT_LIST_DIR}/api/esp_ble_cte_api.c"
    )
endif()

# TODO: Added this file in the ble mesh cmake file
if(CONFIG_BLE_MESH)
    list(APPEND bluedroid_host_srcs "${CMAKE_CURRENT_LIST_DIR}/../../esp_ble_mesh/core/bluedroid_host/adapter.c")
endif()

list(APPEND bluedroid_host_include_dirs
    ${CMAKE_CURRENT_LIST_DIR}/api/include/api
)

list(APPEND bluedroid_host_priv_include_dirs
    ${CMAKE_CURRENT_LIST_DIR}/bta/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/ar/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/av/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/dm/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/gatt/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/hf_ag/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/hf_client/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/hd/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/hh/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/jv/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/pba/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/sdp/include
    ${CMAKE_CURRENT_LIST_DIR}/bta/sys/include
    ${CMAKE_CURRENT_LIST_DIR}/device/include
    ${CMAKE_CURRENT_LIST_DIR}/hci/include
    ${CMAKE_CURRENT_LIST_DIR}/btc/profile/esp/include
    ${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/include
    ${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/include
    ${CMAKE_CURRENT_LIST_DIR}/btc/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/btm/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/gap/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/gatt/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/hid/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/l2cap/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/sdp/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/smp/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/avct/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/avrc/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/avdt/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/rfcomm/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/obex/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/goep/include
    ${CMAKE_CURRENT_LIST_DIR}/stack/include
    ${CMAKE_CURRENT_LIST_DIR}/common/include
    ${CMAKE_CURRENT_LIST_DIR}/config/include
)

if((CONFIG_BT_A2DP_ENABLE AND NOT CONFIG_BT_A2DP_USE_EXTERNAL_CODEC) OR
   (CONFIG_BT_HFP_ENABLE AND CONFIG_BT_HFP_AUDIO_DATA_PATH_HCI AND NOT CONFIG_BT_HFP_USE_EXTERNAL_CODEC))
    list(APPEND bluedroid_host_priv_include_dirs
        ${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/include
        ${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/include
        ${CMAKE_CURRENT_LIST_DIR}/external/sbc/plc/include
    )

    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/alloc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/bitalloc-sbc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/bitalloc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/bitstream-decode.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/decoder-oina.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/decoder-private.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/decoder-sbc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/dequant.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/framing-sbc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/framing.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/oi_codec_version.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/synthesis-8-generated.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/synthesis-dct8.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/decoder/srce/synthesis-sbc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_analysis.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_dct.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_dct_coeffs.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_enc_bit_alloc_mono.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_enc_bit_alloc_ste.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_enc_coeffs.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_encoder.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/encoder/srce/sbc_packing.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/sbc/plc/sbc_plc.c")
endif()

# ============================================================================
# Extended A2DP codec framework + vendor decoders (ported from v5.5.2 fork)
# ============================================================================
if(CONFIG_BT_A2DP_ENABLE)
    # Framework core (decoder-interface + codec DB + vendor dispatch)
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_sbc_decoder.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_codec_config.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor.c")
    list(APPEND bluedroid_host_priv_include_dirs "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/include")
endif()

if(CONFIG_BT_A2DP_APTX_DECODER)
    list(APPEND bluedroid_host_priv_include_dirs ${CMAKE_CURRENT_LIST_DIR}/external/libfreeaptx/main)
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/external/libfreeaptx/main/freeaptx.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_aptx.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_aptx_hd.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_aptx_ll.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_aptx_decoder.c")
endif()

if(CONFIG_BT_A2DP_LDAC_DECODER)
    list(APPEND bluedroid_host_priv_include_dirs
        ${CMAKE_CURRENT_LIST_DIR}/external/libldac-dec/src
        ${CMAKE_CURRENT_LIST_DIR}/external/libldac-dec/inc)
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/external/libldac-dec/src/ldacBT.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libldac-dec/src/ldaclib.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_ldac.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_ldacbt_decoder.c")
endif()

if(CONFIG_BT_A2DP_OPUS_DECODER)
    list(APPEND bluedroid_host_priv_include_dirs
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/include"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/silk"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/silk/fixed"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/silk/float"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/celt"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus/dnn"
        "${CMAKE_CURRENT_LIST_DIR}/external/opus_config")
    file(GLOB opus_srcs  "${CMAKE_CURRENT_LIST_DIR}/external/opus/src/*.c")
    file(GLOB opus_silk  "${CMAKE_CURRENT_LIST_DIR}/external/opus/silk/*.c")
    file(GLOB opus_celt  "${CMAKE_CURRENT_LIST_DIR}/external/opus/celt/*.c")
    file(GLOB opus_fixed "${CMAKE_CURRENT_LIST_DIR}/external/opus/silk/fixed/*.c")
    list(APPEND bluedroid_host_srcs ${opus_srcs} ${opus_silk} ${opus_celt} ${opus_fixed}
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_opus.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_opus_decoder.c")
    set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/external/opus/silk/NLSF2A.c          PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)
    set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/external/opus/silk/quant_LTP_gains.c PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)
    set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/external/opus/celt/celt_lpc.c        PROPERTIES COMPILE_FLAGS -Wno-maybe-uninitialized)
endif()

if(CONFIG_BT_A2DP_LC3PLUS_DECODER)
    list(APPEND bluedroid_host_priv_include_dirs "${CMAKE_CURRENT_LIST_DIR}/external/liblc3/include")
    file(GLOB lc3_srcs "${CMAKE_CURRENT_LIST_DIR}/external/liblc3/src/*.c")
    list(APPEND bluedroid_host_srcs ${lc3_srcs}
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_lc3plus.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_lc3plus_decoder.c")
endif()

if(CONFIG_BT_A2DP_LHDCV5_DECODER)
    list(APPEND bluedroid_host_priv_include_dirs "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5")
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_bit_reader.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_dec.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_entropy_dec.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_imdct.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_sns_synth.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_tables.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_diag_config.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_lhdcv5.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_vendor_lhdcv5_decoder.c")
    set_source_files_properties(
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_bit_reader.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_dec.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_entropy_dec.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_imdct.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_sns_synth.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/lhdc-v5/lhdc_tables.c"
        PROPERTIES COMPILE_OPTIONS "-O3")
endif()

if(CONFIG_BT_A2DP_AAC_DECODER)
    list(APPEND bluedroid_host_priv_include_dirs
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac")
    # Explicit AAC-LC core sources only (NO SBR — sbr.c pulls missing hlxclib headers)
    list(APPEND bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/aacdec.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/aactabs.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/bitstream.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/buffers.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/dct4.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/decelmnt.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/dequant.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/fft.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/filefmt.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/huffman.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/hufftabs.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/imdct.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/noiseless.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/pns.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/stproc.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/tns.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/libhelix_aac/trigtabs.c"
        "${CMAKE_CURRENT_LIST_DIR}/external/libaac-lc/aac_decoder_helix.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_aac.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/latm_parser.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2dp_aac_decoder_custom.c")
endif()

# In a BLE-only build (Classic BT / A2DP disabled) the A2DP core files compile to
# empty (their bodies are guarded by BTC_AV*_INCLUDED), but a few pull in A2DP-only
# headers (e.g. oi_status.h) whose include paths are only added when A2DP is on.
# They define no symbols when A2DP is off, so drop them from the build entirely.
if(NOT CONFIG_BT_A2DP_ENABLE)
    list(REMOVE_ITEM bluedroid_host_srcs
        "${CMAKE_CURRENT_LIST_DIR}/api/esp_a2dp_api.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/bta_av_co.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_control.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_sink.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_sink_ext_coedc.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_latm_raw.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_source.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_a2dp_source_ext_codec.c"
        "${CMAKE_CURRENT_LIST_DIR}/btc/profile/std/a2dp/btc_av.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_api.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_sbc.c"
        "${CMAKE_CURRENT_LIST_DIR}/stack/a2dp/a2d_m24.c")
endif()

# Export the variables to the parent scope
set(bluedroid_host_srcs "${bluedroid_host_srcs}" PARENT_SCOPE)
set(bluedroid_host_include_dirs "${bluedroid_host_include_dirs}" PARENT_SCOPE)
set(bluedroid_host_priv_include_dirs "${bluedroid_host_priv_include_dirs}" PARENT_SCOPE)
