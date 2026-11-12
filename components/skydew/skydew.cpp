#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "skydew.h"

#ifdef USE_ESP32

namespace esphome {
namespace skydew {

static const char *const TAG = "SkyDew";

float SkyDew::get_setup_priority() const { return setup_priority::AFTER_BLUETOOTH; }

void SkyDew::setup() {
  auto err = esp_ble_gattc_app_register(this->app_id);
  if (err) {
    ESP_LOGE(TAG, "gattc app register failed. app_id=%d code=0x%X", this->app_id, err);
    this->mark_failed();
  }
  this->set_state(r4s::DrvState::IDLE);
  this->dew_state.type = this->type;
  this->update_rssi_period = 30;
}

void SkyDew::dump_config() {
  ESP_LOGCONFIG(TAG, "SkyDew:");
  ESP_LOGCONFIG(TAG, "  Model: %s", this->usr_model.c_str());
  ESP_LOGCONFIG(TAG, "  MAC Address: %s", this->parent()->address_str(this->address).c_str());
}

void SkyDew::device_online_() {
  this->clear_pending_commands_();
  this->send_(0xFF);
}

void SkyDew::device_offline_() {
  if (this->status_ind_ != nullptr)
    this->status_ind_->publish_state("Off Line");
  this->is_authorize = false;
  this->is_ready = false;
}

void SkyDew::sync_data_() {
  if((this->signal_strength_ != nullptr) && (this->sync_data_time > this->update_rssi_time)) {
    this->signal_strength_->publish_state(this->rssi);
    this->update_rssi_time = this->sync_data_time + this->update_rssi_period;  
  }
  if(this->is_authorize && this->is_ready) {
    uint8_t cmd;
    if (this->dequeue_command_(cmd)) {
      this->send_(cmd);
    }
    else if (this->needs_clock_sync_())
      this->send_(0x6E);
    else
      this->send_(0x06);
  }
}

void SkyDew::verify_contig_() {
  this->conf_error = true;
  size_t pos = this->mnf_model.find("RHF-");
  if (pos != std::string::npos)
    this->conf_error = false;
  pos = this->mnf_model.find("RSP-");
  if (pos != std::string::npos)
    this->conf_error = false;
  if(this->conf_error)
    ESP_LOGE(TAG, "!!! %s product and %s component are not compatible !!!", this->mnf_model.c_str(), TAG);
}

void SkyDew::parse_response_(uint8_t *data, int8_t data_len, uint32_t timestamp) {
  if (!this->valid_response_(data, data_len, this->status_frame_minimum_())) {
    ESP_LOGW(TAG, "Ignoring malformed/short Ready4Sky response (%d bytes)", data_len);
    return;
  }
  this->is_ready = false;
  if(data[2] != 0x06)
    ESP_LOGD(TAG, "%s NOTIFY: %s", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
  bool err = false;
  switch(data[2]) {
    case 0x01: {
      if((data[1] == this->cmd_count) && data[3]) {
        this->dew_state.version = data[3];
        this->dew_state.relise = data[4];
        ESP_LOGI(TAG, "%s INFO:   Version: %2.2f", this->mnf_model.c_str(), (data[3] + data[4]*0.01));
        this->send_(this->notify_data_time > 0 ? 0x6E : 0x47);
      }
      else
        err = true;
      break;
    }
    case 0x03: { // power ON
      if((data[1] == this->cmd_count) && data[3]) {
        this->send_(0x06);
      }
      else
        err = true;
      break;
    }
    case 0x04: { // power OFF
      if((data[1] == this->cmd_count) && data[3]) {
        this->send_(0x47);
      }
      else
        err = true;
      break;
    }
    case 0x06: {
      this->indication.update = false;
      // опрос сенсоров
      if(this->dew_state.humidity != data[12]) {
        this->dew_state.humidity = data[12];
        ESP_LOGI(TAG, "%s NOTIFY: %s (humidity)", this->mnf_model.c_str(),
                    format_hex_pretty(data, data_len).c_str());
        this->humidity_->publish_state(this->dew_state.humidity);
        this->indication.update = true;
      }
      if(this->dew_state.temperature != data[14]) {
        this->dew_state.temperature = data[14];
        if(this->dew_state.type & 0x06) { // only for RHF-3317S, RHF-3318S, RHF-3320S
          ESP_LOGI(TAG, "%s NOTIFY: %s (temperature)", this->mnf_model.c_str(),
                    format_hex_pretty(data, data_len).c_str());
          if(this->temperature_ != nullptr)
            this->temperature_->publish_state(this->dew_state.temperature);
          this->indication.update = true;
        }
      }
      // обновление режима
      if(this->dew_state.mode != data[3]) {
        this->dew_state.mode = data[3];
        ESP_LOGI(TAG, "%s NOTIFY: %s (mode)", this->mnf_model.c_str(),
                    format_hex_pretty(data, data_len).c_str());
        if(this->dew_state.type & 0x06) { // different for RHF-3317S, RHF-3318S, RHF-3320S
          if(this->auto_mode_ != nullptr) {
            if(this->dew_state.mode == 0x00)
              this->auto_mode_->publish_state(false);
            else
              this->auto_mode_->publish_state(true);
          }
        }
        else { // different for RHF-3310S
          if(this->mode_ != nullptr) {
            this->mode_->publish_state(this->dew_state.mode);
          }
        }
      }
      // обновление состояния выключателя
      if(this->dew_state.status != data[11]) {
        this->dew_state.status = data[11];
        ESP_LOGI(TAG, "%s NOTIFY: %s (state)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
        if(this->dew_state.status == 0x00)
          this->power_->publish_state(false);
        else
          this->power_->publish_state(true);
        this->indication.update = true;
      }
      // обновление состояния ночного режима
      if(this->dew_state.night_mode != data[16]) {
        this->dew_state.night_mode = data[16];
        if(this->dew_state.type & 0x06) { // only for RHF-3317S, RHF-3318S, RHF-3320S
          ESP_LOGI(TAG, "%s NOTIFY: %s (night mode)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
          if(this->night_mode_ != nullptr) {
            if(this->dew_state.night_mode != 0x01)
              this->night_mode_->publish_state(false);
            else
              this->night_mode_->publish_state(true);
          }
        }
      }
      // обновление состояния "тёплого пара"
      if(this->dew_state.warm_steam != data[10]) {
        this->dew_state.warm_steam = data[10];
        if(this->dew_state.type & 0x01) { // only for RHF-3310S
          ESP_LOGI(TAG, "%s NOTIFY: %s (warm_steam)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
          if(this->warm_steam_ != nullptr) {
            if(this->dew_state.warm_steam != 0x01)
              this->warm_steam_->publish_state(false);
            else
              this->warm_steam_->publish_state(true);
          }
        }
      }
      // обновление состояния зумера
      if(this->dew_state.beeper != data[17]) {
        this->dew_state.beeper = data[17];
        ESP_LOGI(TAG, "%s NOTIFY: %s (beeper)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
        if(this->beeper_ != nullptr) {
          if(this->dew_state.beeper != 0x01)
            this->beeper_->publish_state(false);
          else
            this->beeper_->publish_state(true);
        }
      }
      // обновление состояния уровня тумана
      if(this->dew_state.steam_level != data[4]) {
        this->dew_state.steam_level = data[4];
        ESP_LOGI(TAG, "%s NOTIFY: %s (steam level)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
        if(this->steam_level_ != nullptr) { 
          this->steam_level_->publish_state(this->dew_state.steam_level + 1);
        }
      }
      // обновление состояния целевой влажности
      if(this->dew_state.target_humidity != data[5]) {
        this->dew_state.target_humidity = data[5];
        ESP_LOGI(TAG, "%s NOTIFY: %s (target humidity)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
        if(this->target_humidity_ != nullptr) { 
          this->target_humidity_->publish_state(this->dew_state.target_humidity);
        }
      }
      // обновление нештатных состояний
      if(this->dew_state.alarm != data[18]) {
        this->dew_state.alarm = data[18];
        ESP_LOGI(TAG, "%s NOTIFY: %s (alarm)", this->mnf_model.c_str(),
              format_hex_pretty(data, data_len).c_str());
        this->indication.update = true;
      }
      if(this->dew_state.time_min != data[9]) {
        this->dew_state.time_min = data[9];
        this->indication.update = true;
      }
      // индикация
      if ((this->status_ind_ != nullptr) && this->indication.update) {
        this->indication.update = false;
        this->indication.text_power = (this->dew_state.status != 0x00) ? 
              (this->dew_state.language ? "Вкл." : "On"):
              (this->dew_state.language ? "Выкл." : "Off");
        if(this->dew_state.type & 0x01) { // different for RHF-3310S
          if ((this->dew_state.status == 0x20) || (this->dew_state.status == 0x03))
            this->indication.text_condition = (this->dew_state.status == 0x03) ? 
                (this->dew_state.language ? "Нет воды" : "No Water"):
                (this->dew_state.language ? "Замена фильтра" : "Replacing the filter");
        }
        else { // different for RHF-3317S, RHF-3318S, RHF-3320S
          if (this->dew_state.alarm != 0x00)
            this->indication.text_condition = 
                (this->dew_state.alarm == 0x04) ? 
                    (this->dew_state.language ? "Нет воды" : "No Water"):
                    ((this->dew_state.alarm == 0x03) ?
                        (this->dew_state.language ? "Очень влажно (>80%)" : "Very humid (>80%)") :
                        (this->dew_state.language ? ("Ошибка: " + to_string(this->dew_state.alarm)) :
                            ("Error: "+ to_string(this->dew_state.alarm))));
        }
        this->indication.text_time = ((data[8] < 10) ? "0" : "") + to_string(data[8]) + ":" +
                                     ((data[9] < 10) ? "0" : "") + to_string(data[9]);
        this->status_ind_->publish_state( 
            this->indication.text_power + " " +
            this->indication.text_condition + " " +
            this->indication.text_time);
      }
      this->is_ready = true;
      break;
    }
    case 0x05:
    case 0x09:
    case 0x0A:
    case 0x0B:
    case 0x16:
    case 0x1B:
    case 0x3C: {
      if((data[1] == this->cmd_count) && data[3]) {
        this->send_(0x06);
      }
      else
        err = true;
      break;
    }
    case 0x47: {
      if(data[1] == this->cmd_count) {
        // получение времени наработки
        uint32_t val = r4s::read_le32(&data[5]);
        ESP_LOGI(TAG, "%s INFO:   Work Time %ld sec", this->mnf_model.c_str(), val);
        if(this->work_time_ != nullptr)
          this->work_time_->publish_state(val/3600);
        // получение циклов включения
        val = r4s::read_le32(&data[13]);
        ESP_LOGI(TAG, "%s INFO:   Work Cycles %ld", this->mnf_model.c_str(), val);
        if(this->work_cycles_ != nullptr)
          this->work_cycles_->publish_state(val);
        // получение потреблённой энергии
        val = r4s::read_le32(&data[9]);
        ESP_LOGI(TAG, "%s INFO:   Energy %ld Wh", this->mnf_model.c_str(), val);
        if(this->energy_ != nullptr)
          this->energy_->publish_state((float)val*0.001);
        this->send_(0x06);
      }
      else
        err = true;
      break;
    }
    case 0x6E: { // set time
      if((data[1] == this->cmd_count) && !data[3]) {
        this->send_(0x47);
      }
      else
        err = true;
      break;
    }
    case 0xFF: {
      if((data[1] == this->cmd_count) && data[3]) {
        this->is_authorize = true;
        ESP_LOGI(TAG, "%s INFO:   Autorized.", this->mnf_model.c_str());
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
  if(err) {
    ESP_LOGE(TAG, "Error command 0x%02X", data[2]);
    this->send_(0x06);
  }
}

void SkyDew::send_(uint8_t command) {
  this->is_ready = false;
  if(++this->cmd_count == 0)
    this->cmd_count = 1;
  this->send_data_len = 0;
  this->send_data[0] = 0x55;
  this->send_data[1] = this->cmd_count;
  this->send_data[2] = command;
  switch(command) {
    case 0x01:
    case 0x03:
    case 0x04:
    case 0x06:{
      this->send_data_len = 4;
      break;
    }
    case 0x05:{
      if(this->dew_state.type & 0x01)
        this->send_data_len = 10;
      else
        this->send_data_len = 4;
      break;
    }
    case 0x09:
    case 0x0A:
    case 0x16:
    case 0x1B:
    case 0x3C:{
      this->send_data_len = 5;
      break;
    }
    case 0x0B:{
      this->send_data_len = 5;
      if(this->dew_state.type & 0x06)
        this->send_data_len = 6;
      break;
    }
    case 0x47:{
      this->send_data[3] = 0x00;
      this->send_data_len = 5;
      break;
    }
    case 0x6E:{
      this->send_data_len = 12;
      memcpy(&this->send_data[3], &this->notify_data_time, 4);
      memcpy(&this->send_data[7], &this->tz_offset, 4);
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

void SkyDew::send_power(bool state) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  if(state) {
    if(this->is_ready && !this->has_pending_commands_())
      this->send_(0x03);
    else
      this->queue_command_(0x03);
  }
  else {
    if(this->is_ready && !this->has_pending_commands_())
      this->send_(0x04);
    else
      this->queue_command_(0x04);
  }
}

void SkyDew::send_auto_mode(bool state) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  if(state)
    this->send_data[3] = 0x01;
  else
    this->send_data[3] = 0x00;
  if(this->is_ready && !this->has_pending_commands_())
    this->send_(0x09);
  else
    this->queue_command_(0x09);
}

void SkyDew::send_night_mode(bool state) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  if(state)
    this->send_data[3] = 0x01;
  else
    this->send_data[3] = 0x00;
  if(this->is_ready && !this->has_pending_commands_())
    this->send_(0x1B);
  else
    this->queue_command_(0x1B);
}

void SkyDew::send_beeper(bool state) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  if(state)
    this->send_data[3] = 0x01;
  else
    this->send_data[3] = 0x00;
  if(this->is_ready && !this->has_pending_commands_())
    this->send_(0x3C);
  else
    this->queue_command_(0x3C);
}

void SkyDew::send_warm_steam(bool state) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  if(state)
    this->send_data[3] = 0x01;
  else
    this->send_data[3] = 0x00;
  if(this->is_ready && !this->has_pending_commands_())
    this->send_(0x16);
  else
    this->queue_command_(0x16);
}

void SkyDew::send_target_humidity(uint8_t value) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  this->target_humidity_->publish_state(value);
  if(this->dew_state.target_humidity != value) {
    this->send_data[3] = value;
    this->send_data[4] = 0x00;
    if(this->is_ready && !this->has_pending_commands_())
      this->send_(0x0B);
    else
      this->queue_command_(0x0B);
  }
}

void SkyDew::send_steam_level(uint8_t value) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  this->steam_level_->publish_state(value);
  if(this->dew_state.steam_level != (value - 1)) {
    this->send_data[3] = value - 1;
    if(this->is_ready && !this->has_pending_commands_())
      this->send_(0x0A);
    else
      this->queue_command_(0x0A);
  }
}

void SkyDew::send_mode(uint8_t value) {
  if (!this->commands_available()) return;
  CommandStagingGuard staging(this);
  this->send_data[3] = value;
  if(this->is_ready && !this->has_pending_commands_())
    this->send_(0x09);
  else
    this->queue_command_(0x09);
}

}  // namespace skykettle
}  // namespace esphome

#endif
