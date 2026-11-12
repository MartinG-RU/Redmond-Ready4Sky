#pragma once
#include <cstdint>
#include <cstddef>
using esp_err_t = int;
using esp_gatt_if_t = int;
using esp_gattc_cb_event_t = int;
using esp_gap_ble_cb_event_t = int;
using esp_ble_addr_type_t = int;
using esp_bd_addr_t = uint8_t[6];
struct esp_bt_uuid_t { uint16_t len; union { uint16_t uuid16; uint32_t uuid32; uint8_t uuid128[16]; } uuid; };
struct esp_ble_gap_cb_param_t { uint8_t bytes[128]{}; };
struct esp_ble_gattc_cb_param_t { uint8_t bytes[128]{}; };
struct esp_ble_scan_params_t { int scan_type, own_addr_type, scan_filter_policy, scan_interval, scan_window, scan_duplicate; };
constexpr int BLE_ADDR_TYPE_PUBLIC = 0;
constexpr int ESP_GATT_IF_NONE = 255;
constexpr int BLE_SCAN_TYPE_ACTIVE = 1;
constexpr int BLE_SCAN_TYPE_PASSIVE = 0;
constexpr int BLE_SCAN_FILTER_ALLOW_ALL = 0;
constexpr int BLE_SCAN_DUPLICATE_DISABLE = 0;
constexpr int ESP_OK = 0;
constexpr int ESP_GATT_OK = 0;
constexpr int ESP_GATT_WRITE_TYPE_RSP = 0;
constexpr int ESP_GATT_AUTH_REQ_NONE = 0;
int esp_ble_gattc_write_char(int, uint16_t, uint16_t, int, const uint8_t *, int, int);
int esp_ble_gap_disconnect(const uint8_t *);
inline int esp_ble_gattc_app_register(int) { return 0; }
