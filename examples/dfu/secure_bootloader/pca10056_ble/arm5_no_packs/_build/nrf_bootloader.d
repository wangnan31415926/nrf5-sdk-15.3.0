  sd_ble_gap_addr_get )� , adv_handle �� , p_addr [�    sd_ble_gap_adv_addr_get �� , pp_wl_addrs ,� , len ��!   sd_ble_gap_whitelist_set �� , pp_id_keys �� , pp_local_irks U� , len ��)   !sd_ble_gap_device_identities_set �� , p_privacy_params �   sd_ble_gap_privacy_set �� , p_privacy_params �   sd_ble_gap_privacy_get �� , p_adv_handle �� , p_adv_data �� , p_adv_params ��%   sd_ble_gap_adv_set_configure J� , adv_handle �� , conn_cfg_tag ��   sd_ble_gap_adv_start �� , adv_handle ��   sd_ble_gap_adv_stop � , conn_handle N� , p_conn_params W�%   sd_ble_gap_conn_param_update Q� , conn_handle N� , hci_status_code ��   sd_ble_gap_disconnect 5� , role ,� , handle e� , 	tx_power `�    sd_ble_gap_tx_power_set 8� , appearance ��"   sd_ble_gap_appearance_set �� , p_appearance ��"   sd_ble_gap_appearance_get �� , p_conn_params W�   sd_ble_gap_ppcp_set � , p_conn_params W�   sd_ble_gap_ppcp_get � , p_write_perm �� , p_dev_name �� , len ��#   sd_ble_gap_device_name_set :� , p_dev_name �� , p_len ��#   sd_ble_gap_device_name_get F� , conn_handle N� , p_sec_params ��    sd_ble_gap_authenticate \� , conn_handle N� , sec_status �� , p_sec_params �� , p_sec_keyset ��$   sd_ble_gap_sec_params_reply �� , conn_handle N� , 	key_type l� , p_key ��"   sd_ble_gap_auth_key_reply r� , conn_handle N� , p_dhkey ��$   sd_ble_gap_lesc_dhkey_reply �� , conn_handle N� , kp_not O�#   sd_ble_gap_keypress_notify �� , conn_handle N� , 	p_pk_own d� , p_oobd_own ��%   sd_ble_gap_lesc_oob_data_get s� , conn_handle N� , p_oobd_own �� , p_oobd_peer =�%   sd_ble_gap_lesc_oob_data_set g� , conn_handle N� , p_master_id 4� , p_enc_info ��   sd_ble_gap_encrypt `� , conn_handle N� , p_enc_info �� , 
p_id_info � , p_sign_info 0�"   sd_ble_gap_sec_info_reply �� , conn_handle N� , p_conn_sec ��    sd_ble_gap_conn_sec_get t� , conn_handle N� , threshold_dbm m� , skip_count ��   sd_ble_gap_rssi_start � , conn_handle N�   sd_ble_gap_rssi_stop {� , conn_handle N� , p_rssi 5� , p_ch_index ��   sd_ble_gap_rssi_get � , p_scan_params `� , p_adv_report_buffer ��   sd_ble_gap_scan_start -�   sd_ble_gap_scan_stop �� , p_peer_addr F� , p_scan_params `� , p_conn_params W� , conn_cfg_tag ��   sd_ble_gap_connect {�"   sd_ble_gap_connect_cancel �� , conn_handle N� , p_gap_phys ��   sd_ble_gap_phy_update ,� , conn_handle N� , p_dl_params 9� , p_dl_limitation {�&   sd_ble_gap_data_length_update �� ,
 interval_us �,   $sd_ble_gap_qos_channel_survey_start �+   #sd_ble_gap_qos_channel_survey_stop ��  P    SYSTEM_SERVICE_ATT_SIZE f� P    INIT_COMMAND_MAX_SIZE 0�! P    INIT_COMMAND_MAX_SIZE_v1 $� P    CODE_PAGE_SIZE ~� P    DATA_OBJECT_MAX_SIZE ��$ P    BOOTLOADER_SETTINGS_ADDRESS u�& P    BOOTLOADER_SETTINGS_PAGE_SIZE ��) P    !NRF_UICR_MBR_PARAMS_PAGE_ADDRESS ��! P    NRF_MBR_PARAMS_PAGE_SIZE <�$ P    NRF_MBR_PARAMS_PAGE_ADDRESS k�+ P    #BOOTLOADER_SETTINGS_BACKUP_ADDRESS R�   _do_assert �� P    DFU_APP_DATA_RESERVED J� P    DFU_REGION_END w� P   BLE_STACK_SUPPORT_REQD �� P    DFU_REGION_START �� P    DFU_REGION_TOTAL_SIZE "� P    NRF_DFU_CURRENT_BANK_0 �� P    NRF_DFU_CURRENT_BANK_1 ��! P    NRF_DFU_BANK_LAYOUT_DUAL :�# P    NRF_DFU_BANK_LAYOUT_SINGLE �� P    NRF_DFU_BANK_INVALID �� P    NRF_DFU_BANK_VALID_APP �� P    NRF_DFU_BANK_VALID_SD =� P    NRF_DFU_BANK_VALID_BL F�! P    NRF_DFU_BANK_VALID_SD_BL J�# P    NRF_DFU_BANK_VALID_EXT_APP �� 8  
<unnamed> Q�  
" �ximage_size (�  
" �x
image_crc ��  
" �x
bank_code ��    nrf_dfu_bank_t 3� 8  semicolon_swallower �� 8  
<unnamed> Q�  
" �xcommand_size B�  
" �xcommand_offset r�  
" �xcommand_crc ��  
" �xdata_object_size �� 8  
<unnamed> S� 8  
<unnamed> Q�  
" �yfirmware_image_crc R�"  
" �yfirmware_image_crc_last 5�   
" �yfirmware_image_offset ��%  
" �yfirmware_image_offset_last �� 8  
<unnamed> Q�  
" �yupdate_start_address J�    dfu_progress_t ��$    NRF_DFU_EVT_DFU_INITIALIZED ��(     NRF_DFU_EVT_TRANSPORT_ACTIVATED -�*    "NRF_DFU_EVT_TRANSPORT_DEACTIVATED ��     NRF_DFU_EVT_DFU_STARTED ��$    NRF_DFU_EVT_OBJECT_RECEIVED ��    NRF_DFU_EVT_DFU_FAILED �"    NRF_DFU_EVT_DFU_COMPLETED �     NRF_DFU_EVT_DFU_ABORTED ��    nrf_dfu_evt_type_t W�    nrf_dfu_observer_t _� P    NRF_DFU_PEER_DATA_LEN ;� P    NRF_DFU_ADV_NAME_LEN �� 8  
<unnamed> Q�  
" �ycrc z�  " �yble_id (�  " �yenc_key ��  <" �ysys_serv_attr ��    nrf_dfu_peer_data_t  �$    DFU_PEER_DATA_STATE_INVALID ��(     DFU_PEER_DATA_STATE_INITIALIZED g�,    $DFU_PEER_DATA_STATE_WRITE_REQUESTED ��+    #DFU_PEER_DATA_STATE_WRITE_FINISHED c�)    !DFU_PEER_DATA_STATE_WRITE_FAILED �"    nrf_dfu_peer_data_state_t �� 8  
<unnamed> Q�  
" �ycrc n�  <" �yname ��  
" �ylen g�    nrf_dfu_adv_name_t ��#    DFU_ADV_NAME_STATE_INVALID ��'    DFU_ADV_NAME_STATE_INITIALIZED ��+    #DFU_ADV_NAME_STATE_WRITE_REQUESTED E�*    "DFU_ADV_NAME_STATE_WRITE_FINISHED ��(     DFU_ADV_NAME_STATE_WRITE_FAILED X�%    nrf_dfu_set_adv_name_state_t M�$ P    SETTINGS_RESERVED_AREA_SIZE S�& P    SETTINGS_BOOT_VALIDATION_SIZE ��    NO_VALIDATION ��    VALIDATE_CRC d�    VALIDATE_SHA256 ��#    VALIDATE_ECDSA_P256_SHA256 ��    boot_validation_type_t �� 8  
<unnamed> Q�  "" �ytype ��  <" @�ybytes ��    boot_validation_t �� 8  
<unnamed> Q�  
" �ycrc X�  
" �ysettings_version ��  
" �yapp_version ��  
" �ybootloader_version ��  
" �ybank_layout ��  
" �ybank_current ��  " �ybank_0 E�  " �ybank_1 D�  
" �ywrite_offset m�  
" �ysd_size ��  "  �y	progress ��  
" �yenter_buttonless_dfu  �  <" ��yinit_command ��  
" �yboot_validation_crc ��%  " A�yboot_validation_softdevice J�  " A�yboot_validation_app C�%  " A�yboot_validation_bootloader K�  " @�y
peer_data ��  " �y	adv_name !�    nrf_dfu_settings_t V� P    NRF_DFU_FLASH_H__ S�!    nrf_dfu_flash_callback_t !� , sd_irq_initialized K�   nrf_dfu_flash_init `� ,
 dest '� , p_src �� ,
 len �� , 	callback ��   nrf_dfu_flash_store �� ,
 
page_addr 6� ,
 
num_pages � , 	callback ��   nrf_dfu_flash_erase �  �s_dfu_settings h� , 	callback ��   nrf_dfu_settings_write ~� , 	callback ��    nrf_dfu_settings_backup 1� , 	callback ��*   "nrf_dfu_settings_write_and_backup � , sd_irq_initialized K�   nrf_dfu_settings_init �� , p_data \�)   !nrf_dfu_settings_peer_data_write f� , p_data \�(    nr