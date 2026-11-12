#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
using esp_err_t = int;
using esp_gatt_if_t = int;
using esp_gattc_cb_event_t = int;
using esp_gap_ble_cb_event_t = int;
using esp_ble_addr_type_t = int;
using esp_gatt_status_t = int;
using esp_ble_auth_req_t = uint8_t;
using esp_ble_io_cap_t = uint8_t;
using esp_bd_addr_t = uint8_t[6];
struct esp_bt_uuid_t { uint16_t len; union { uint16_t uuid16; uint32_t uuid32; uint8_t uuid128[16]; } uuid; };
struct esp_ble_gap_cb_param_t {
  struct { int status; } local_privacy_cmpl{}, scan_param_cmpl{}, scan_start_cmpl{}, scan_stop_cmpl{};
  struct { int search_evt; uint8_t bda[6]; uint8_t ble_adv[64]; int rssi; int ble_addr_type; } scan_rst{};
  struct { struct { bool success; uint8_t fail_reason, auth_mode; } auth_cmpl; struct { uint8_t key_type; } ble_key; } ble_security{};
};
struct esp_ble_gattc_cb_param_t {
  struct { int app_id; int status; } reg{};
  struct { int status; uint16_t conn_id; uint8_t remote_bda[6]; } open{}, close{};
  struct { int status; uint16_t conn_id; } cfg_mtu{}, search_cmpl{};
  struct { uint16_t conn_id, start_handle, end_handle; } search_res{};
  struct { int status; uint16_t conn_id, handle; } write{};
  struct { uint16_t conn_id; uint8_t remote_bda[6]; uint16_t handle, value_len; uint8_t *value; bool is_notify; } notify{};
  struct { int status; uint16_t handle; } reg_for_notify{};
  struct { uint16_t conn_id; uint8_t remote_bda[6]; } connect{}, disconnect{};
};
struct esp_ble_scan_params_t { int scan_type, own_addr_type, scan_filter_policy, scan_interval, scan_window, scan_duplicate; };
struct esp_gattc_char_elem_t { uint16_t char_handle; uint8_t properties; };
struct esp_gattc_descr_elem_t { esp_bt_uuid_t uuid; uint16_t handle; };
struct esp_ble_gattc_cancel_open_params_t { esp_gatt_if_t gattc_if; uint8_t remote_bda[6]; };
struct esp_bt_controller_config_t { int unused; };
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() esp_bt_controller_config_t{}
constexpr int BLE_ADDR_TYPE_PUBLIC=0, ESP_GATT_IF_NONE=255, BLE_SCAN_TYPE_ACTIVE=1, BLE_SCAN_TYPE_PASSIVE=0;
constexpr int BLE_SCAN_FILTER_ALLOW_ALL=0, BLE_SCAN_DUPLICATE_DISABLE=0, ESP_OK=0, ESP_GATT_OK=0;
constexpr int ESP_GATT_WRITE_TYPE_RSP=0, ESP_GATT_AUTH_REQ_NONE=0, ESP_BD_ADDR_LEN=6;
constexpr int ESP_UUID_LEN_16=2, ESP_UUID_LEN_32=4, ESP_UUID_LEN_128=16, ESP_GATT_UUID_CHAR_CLIENT_CONFIG=0x2902;
constexpr int ESP_GATT_DB_CHARACTERISTIC=0, ESP_GATT_DB_DESCRIPTOR=1;
constexpr int ESP_GATT_CHAR_PROP_BIT_WRITE=0x08, ESP_GATT_CHAR_PROP_BIT_NOTIFY=0x10;
constexpr int ESP_BT_STATUS_SUCCESS=0, ESP_BT_CONTROLLER_STATUS_IDLE=0, ESP_BT_CONTROLLER_STATUS_INITED=1;
constexpr int ESP_BT_CONTROLLER_STATUS_ENABLED=2, ESP_BT_MODE_BLE=1, ESP_BT_MODE_CLASSIC_BT=2;
constexpr int ESP_LE_AUTH_REQ_SC_ONLY=8, ESP_IO_CAP_NONE=0, ESP_BLE_SEC_ENCRYPT=1;
constexpr int ESP_BLE_SM_AUTHEN_REQ_MODE=0, ESP_BLE_SM_IOCAP_MODE=1, ESP_BLE_AD_TYPE_NAME_CMPL=9;
enum { ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT=1, ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT,
  ESP_GAP_BLE_SCAN_START_COMPLETE_EVT, ESP_GAP_BLE_SCAN_RESULT_EVT, ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT,
  ESP_GAP_BLE_AUTH_CMPL_EVT, ESP_GAP_BLE_KEY_EVT };
enum { ESP_GAP_SEARCH_INQ_RES_EVT=1, ESP_GAP_SEARCH_INQ_CMPL_EVT };
enum { ESP_GATTC_REG_EVT=1, ESP_GATTC_CONNECT_EVT, ESP_GATTC_OPEN_EVT, ESP_GATTC_CFG_MTU_EVT,
  ESP_GATTC_SEARCH_RES_EVT, ESP_GATTC_SEARCH_CMPL_EVT, ESP_GATTC_REG_FOR_NOTIFY_EVT,
  ESP_GATTC_WRITE_DESCR_EVT, ESP_GATTC_WRITE_CHAR_EVT, ESP_GATTC_NOTIFY_EVT,
  ESP_GATTC_DISCONNECT_EVT, ESP_GATTC_CLOSE_EVT };
extern int fake_stop_calls, fake_open_calls, fake_disconnect_calls, fake_close_calls;
extern int fake_search_calls, fake_write_calls, fake_cancel_calls;
inline int nvs_flash_init() { return ESP_OK; }
inline int esp_bt_controller_get_status() { return ESP_BT_CONTROLLER_STATUS_ENABLED; }
inline int esp_bt_controller_init(esp_bt_controller_config_t *) { return ESP_OK; }
inline int esp_bt_controller_enable(int) { return ESP_OK; }
inline int esp_bt_controller_mem_release(int) { return ESP_OK; }
inline int esp_bluedroid_init() { return ESP_OK; }
inline int esp_bluedroid_enable() { return ESP_OK; }
inline int esp_ble_gap_register_callback(void (*)(int, esp_ble_gap_cb_param_t *)) { return ESP_OK; }
inline int esp_ble_gattc_register_callback(void (*)(int, int, esp_ble_gattc_cb_param_t *)) { return ESP_OK; }
inline int esp_ble_gap_set_scan_params(esp_ble_scan_params_t *) { return ESP_OK; }
inline int esp_ble_gap_start_scanning(uint32_t) { return ESP_OK; }
inline int esp_ble_gap_stop_scanning() { ++fake_stop_calls; return ESP_OK; }
inline uint8_t *esp_ble_resolve_adv_data(uint8_t *adv, int, uint8_t *len) { *len=adv[0]; return adv+1; }
inline int esp_ble_gattc_open(int, const uint8_t *, int, bool) { ++fake_open_calls; return ESP_OK; }
inline int esp_ble_gattc_close(int, uint16_t) { ++fake_close_calls; return ESP_OK; }
inline int esp_ble_gattc_cancel_open(esp_ble_gattc_cancel_open_params_t *) { ++fake_cancel_calls; return ESP_OK; }
inline int esp_ble_gap_set_security_param(int, void *, uint8_t) { return ESP_OK; }
inline int esp_ble_set_encryption(const uint8_t *, int) { return ESP_OK; }
inline int esp_ble_gattc_send_mtu_req(int, uint16_t) { return ESP_OK; }
inline int esp_ble_gattc_search_service(int, uint16_t, esp_bt_uuid_t *) { ++fake_search_calls; return ESP_OK; }
inline int esp_ble_gattc_get_attr_count(int, uint16_t, int, uint16_t, uint16_t, uint16_t, uint16_t *count) { *count=2; return ESP_OK; }
inline int esp_ble_gattc_get_char_by_uuid(int, uint16_t, uint16_t, uint16_t, esp_bt_uuid_t uuid, esp_gattc_char_elem_t *out, uint16_t *count) {
  *count=1; const bool tx=uuid.uuid.uuid128[12]==2; out->char_handle=tx?10:11;
  out->properties=tx?ESP_GATT_CHAR_PROP_BIT_WRITE:ESP_GATT_CHAR_PROP_BIT_NOTIFY; return ESP_OK;
}
inline int esp_ble_gattc_register_for_notify(int, const uint8_t *, uint16_t) { return ESP_OK; }
inline int esp_ble_gattc_get_descr_by_char_handle(int, uint16_t, uint16_t, esp_bt_uuid_t, esp_gattc_descr_elem_t *out, uint16_t *count) {
  *count=1; out->handle=12; out->uuid.len=ESP_UUID_LEN_16; out->uuid.uuid.uuid16=ESP_GATT_UUID_CHAR_CLIENT_CONFIG; return ESP_OK;
}
inline int esp_ble_gattc_write_char_descr(int, uint16_t, uint16_t, int, uint8_t *, int, int) { return ESP_OK; }
inline int esp_ble_gattc_write_char(int, uint16_t, uint16_t, int, const uint8_t *, int, int) { ++fake_write_calls; return ESP_OK; }
inline int esp_ble_gap_disconnect(const uint8_t *) { ++fake_disconnect_calls; return ESP_OK; }
inline int esp_ble_gattc_app_register(int) { return ESP_OK; }
