#ifdef USE_ESP32
#include "ready4sky.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome::ready4sky {
static const char *const TAG = "Ready4Sky";

bool R4SDriver::queue_command_(uint8_t command) {
  if (!this->is_authorize) return false;
  // A queued OFF cancels earlier unsent requests, including ON.
  if (command == 0x04) this->pending_.clear();
  if (!this->pending_.push(command, this->send_data)) {
    ESP_LOGW(TAG, "%s: command queue full; rejected 0x%02X", this->usr_model.c_str(), command);
    return false;
  }
  return true;
}

void R4SDriver::write_command_(uint8_t command) {
  if ((!this->is_authorize && command != 0xFF) || this->send_data_len < 4 || this->send_data_len > BLE_BUFF_SIZE)
    return;
  if (command == 0xFF && ++this->auth_attempts_ > 3) {
    ESP_LOGW(TAG, "%s: authentication failed repeatedly", this->usr_model.c_str());
    this->request_disconnect_();
    return;
  }
  const esp_err_t status = esp_ble_gattc_write_char(this->gattc_if, this->conn_id, this->tx_char_handle,
      this->send_data_len, this->send_data, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);
  if (status != ESP_OK) {
    ESP_LOGE(TAG, "%s: failed to send command 0x%02X (0x%X)", this->usr_model.c_str(), command, status);
    this->request_disconnect_();
    return;
  }
  ++this->tx_generation_;
  this->waiting_response_ = true;
  this->expected_sequence_ = this->send_data[1];
  this->expected_command_ = this->send_data[2];
  this->command_sent_ms_ = millis();
  if (this->expected_command_ == 0x6E) {
    this->last_clock_attempt_ms_ = this->command_sent_ms_ ? this->command_sent_ms_ : 1;
    this->sent_clock_offset_ = this->tz_offset;
  }
  if (this->expected_command_ != 0x06 && this->expected_command_ != 0xFF)
    ESP_LOGD(TAG, "%s SEND: %s", this->mnf_model.c_str(),
             format_hex_pretty(this->send_data, this->send_data_len).c_str());
}

void R4SDriver::check_timeout_(uint32_t now) {
  if (!this->waiting_response_ || now - this->command_sent_ms_ < 15000) return;
  ESP_LOGW(TAG, "%s: response timeout for 0x%02X; disconnecting", this->usr_model.c_str(),
           this->expected_command_);
  this->request_disconnect_();
}

void R4SDriver::request_disconnect_() {
  this->clear_pending_commands_();
  this->device_offline_();
  this->set_state(DrvState::DISCONNECTING);
  if (esp_ble_gap_disconnect(this->remote_bda) != ESP_OK) {
    this->set_state(DrvState::IDLE);
    if (this->parent() != nullptr) this->parent()->start_scan();
  }
}

}  // namespace esphome::ready4sky
#endif
