#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include <string>
#include "skysmoke.h"

#ifdef USE_ESP32

namespace esphome {
namespace skysmoke {

static const char *const TAG = "SkySmoke";

float SkySmoke::get_setup_priority() const { return setup_priority::AFTER_BLUETOOTH; }

void SkySmoke::setup() {
  auto err = esp_ble_gattc_app_register(this->app_id);
  if (err) {
    ESP_LOGE(TAG, "gattc app register failed. app_id=%d code=0x%X", this->app_id, err);
    this->mark_failed();
  }
  this->set_state(r4s::DrvState::IDLE);
  this->smoke_state.type = this->type;
  
}

void SkySmoke::dump_config() {
  ESP_LOGCONFIG(TAG, "SkySmoke:");
  ESP_LOGCONFIG(TAG, "  Model: %s (type %d)", this->usr_model.c_str(), this->type);
  ESP_LOGCONFIG(TAG, "  MAC Address: %s", this->parent()->address_str(this->address).c_str());
}

void SkySmoke::device_online_() {
  this->send_(0xFF);
}

void SkySmoke::device_offline_() {
  this->is_authorize = false;
  this->is_active = false;
  if (this->smoke_ != nullptr) this->smoke_->invalidate_state();
  if (this->temperature_ != nullptr) this->temperature_->publish_state(NAN);
  if (this->battery_level_ != nullptr) this->battery_level_->publish_state(NAN);
  if (this->signal_strength_ != nullptr) this->signal_strength_->publish_state(NAN);
  this->smoke_state.smoke = 0xFF;
  this->smoke_state.temperature_raw = 0xFFFF;
  this->smoke_state.battery_level = 0xFF;
}

void SkySmoke::sync_data_() {
  if(this->is_authorize && this->is_active)
    this->send_(0x06);
}

void SkySmoke::verify_contig_() {
  size_t pos = this->mnf_model.find("RSS");
  if (pos != std::string::npos)
    this->conf_error = false;
  pos = this->mnf_model.find("RFS-HSS");
  if (pos != std::string::npos)
    this->conf_error = false;
  if(this->conf_error)
    ESP_LOGE(TAG, "!!! %s product and %s component are not compatible !!!", this->mnf_model.c_str(), TAG);
}

void SkySmoke::parse_response_(uint8_t *data, int8_t data_len, uint32_t timestamp) {
  if (!this->valid_response_(data, data_len, this->status_frame_minimum_())) {
    ESP_LOGW(TAG, "Ignoring malformed/short Ready4Sky response (%d bytes)", data_len);
    return;
  }
  ESP_LOGD(TAG, "%s notify value: %s", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
  if((this->signal_strength_ != nullptr) && (timestamp >= this->update_rssi_time)) {
    this->signal_strength_->publish_state(this->rssi);
    this->update_rssi_time = timestamp + this->update_rssi_period;  
  }

  switch(data[2]) {
    case 0x01: {
      if((data[1] == this->cmd_count) && data[3]) {
        this->smoke_state.version = data[3];
        this->smoke_state.relise = data[4];
        ESP_LOGI(TAG, "Version: %2.2f", (data[3] + data[4]*0.01));
        this->send_(0x06);
      }
      break;
    }
    case 0x06: {
      // обнаружение задымления
      if(this->smoke_state.smoke != data[12]) {
        this->smoke_state.smoke = data[12];
        this->smoke_->publish_state(this->smoke_state.smoke);
      }
      // обновление заряда батарей
      if(this->smoke_state.battery_level != data[6]) {
        this->smoke_state.battery_level = data[6];
        if(this->battery_level_ != nullptr)
          this->battery_level_->publish_state(this->smoke_state.battery_level);
      }
      // обновление температура
      const uint16_t raw_temperature = (static_cast<uint16_t>(data[5]) << 8) | data[4];
      if(this->smoke_state.temperature_raw != raw_temperature) {
        this->smoke_state.temperature_raw = raw_temperature;
        if(this->temperature_ != nullptr)
          this->temperature_->publish_state(((((uint16_t)data[5] & 0x000F)<<4) + (uint16_t)data[4]) * 0.1);
      }
      if(this->smoke_state.type & 0x01) {
        this->is_active = true;
        this->sync_data_period = 10;
      }
      break;
    }
    case 0xFF: {
      if((data[1] == this->cmd_count) && data[3]) {
        this->is_authorize = true;
        ESP_LOGI(TAG, "%s autorized.", this->mnf_model.c_str());
        this->send_(0x01);
      }
      else
        this->send_(0xFF);
      break;
    }
    default: {
      break;
    }
  }
}

void SkySmoke::send_(uint8_t command) {
  this->is_active = false;
  if(++this->cmd_count == 0)
    this->cmd_count = 1;
  this->send_data_len = 0;
  this->send_data[0] = 0x55;
  this->send_data[1] = this->cmd_count;
  this->send_data[2] = command;
  switch(command) {
    case 0x01:
    case 0x06:{
      this->send_data_len = 4;
      break;
    }
    case 0xFF:{
      this->send_data[3] = 0x2F;
      for (int i = 0; i < 6; i++) 
        this->send_data[i+4] = this->remote_bda[i] ^ this->mnf_model[i];
      this->send_data[10] = 0x2F;
      this->send_data_len = 12;
      break;
    }
    default: {
      this->send_data[2] = 0x06;
      this->send_data_len = 4;
      break;
    }
  }
  this->send_data[this->send_data_len - 1] = 0xAA;
  this->write_command_(command);
}



}  // namespace skysmoke
}  // namespace esphome

#endif