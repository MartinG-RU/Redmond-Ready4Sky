#pragma once

#include "esphome/core/component.h"
#include "protocol.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esphome/core/defines.h"
#include "esphome/components/time/real_time_clock.h"

#include <array>
#include <span>
#include <string>
#include <vector>
#include <atomic>

#ifdef USE_ESP32

#include <esp_gap_ble_api.h>
#include <esp_gatt_common_api.h>
#include <esp_gattc_api.h>
#include <esp_bt_defs.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

namespace esphome {
namespace ready4sky {

#define BLE_BUFF_SIZE 24
static_assert(BLE_BUFF_SIZE == R4S_FRAME_SIZE, "Ready4Sky buffer size mismatch");

enum class DrvState {
  IDLE, // Connection is idle, no device detected.
  DISCOVERED, // Device advertisement found.
  CONNECTING, // Connection in progress.
  CONNECTED, // Initial connection established.
  ESTABLISHED, // The client and sub-clients have completed setup.
  DISCONNECTING, // Link shutdown is pending; bounded by a watchdog.
};

struct QueuedBLEEvent {
  bool is_gap{false};
  esp_gap_ble_cb_event_t gap_event{};
  esp_gattc_cb_event_t gatt_event{};
  esp_gatt_if_t gatt_if{ESP_GATT_IF_NONE};
  esp_ble_gap_cb_param_t gap{};
  esp_ble_gattc_cb_param_t gatt{};
  uint8_t notify_value[BLE_BUFF_SIZE]{};
};

struct ScanDevice {
  uint64_t address;
  std::string name;
  int rssi;
};


class R4SEngine;

class R4SDriver {
  public:
    virtual void parse_response_(uint8_t *data, int8_t size, uint32_t timestamp) = 0;
    virtual void device_online_() = 0;
    virtual void device_offline_() = 0;
    virtual void sync_data_() = 0;
    virtual void verify_contig_() = 0;
    
    void set_address(uint64_t address) { this->address = address; }
    void set_model(std::string model) { this->usr_model = model; }
    void set_type(uint8_t type) { this->type = type; }
    void set_state(DrvState st) {
      if (this->state_ != st) this->state_since_ms_ = millis();
      this->state_ = st;
    }
    uint32_t state_since_ms() const { return this->state_since_ms_; }
    void check_timeout_(uint32_t now);
    void request_disconnect_();
    virtual size_t status_frame_minimum_() const = 0;
    virtual void hold_if_waiting_() = 0;
    void set_engine_parent(R4SEngine *parent) { this->parent_ = parent; }
    void gattc_event_handler( esp_gattc_cb_event_t event,
                              esp_gatt_if_t gattc_if, 
                              esp_ble_gattc_cb_param_t *param );
    DrvState state() const { return state_; }
    R4SEngine *parent() { return this->parent_; }
    std::string uuid_to_string(esp_bt_uuid_t uuid);

    int app_id{0};
    int gattc_if{ESP_GATT_IF_NONE};
    uint16_t conn_id{0};
    
    esp_bd_addr_t remote_bda{ 0, };
    esp_ble_addr_type_t address_type{BLE_ADDR_TYPE_PUBLIC};
    int rssi{0};
    
    uint16_t service_start_handle{0};
    uint16_t service_end_handle{0};
    uint16_t rx_char_handle{0};
    uint16_t tx_char_handle{0};

    uint64_t address{0};
    std::string usr_model = ""; // User specified model
    std::string mnf_model = ""; // Model specified by the manufacturer
    uint8_t type = 0;
    
    bool    is_authorize = false;
    bool    conf_error = true;
    uint8_t cmd_count = 1;
    uint8_t notify_data[BLE_BUFF_SIZE]{};
    int8_t  notify_data_len = 0;
    int32_t notify_data_time = 0;
    uint8_t send_data[BLE_BUFF_SIZE]{};
    int8_t  send_data_len = 0;
    int32_t sync_data_time = 0;
    int32_t sync_data_period = 1;
    int32_t update_rssi_time = 0;
    int32_t update_rssi_period = 30;
    int32_t tz_offset = 0;

  protected:
    // A public command is accepted only after the first valid status response.
  public:
    bool commands_available() const { return this->is_authorize && this->initialized_; }
    void clear_pending_commands_() {
      this->pending_.clear();
      this->initialized_ = false;
      this->waiting_response_ = false;
      this->clock_synchronized_ = false;
      this->last_clock_attempt_ms_ = 0;
      this->auth_attempts_ = 0;
    }
    void note_status_received_() { this->initialized_ = true; }
  protected:
    // Saves active response metadata while a new command is being prepared.
    class CommandStagingGuard {
     public:
      explicit CommandStagingGuard(R4SDriver *driver) : driver_(driver), generation_(driver->tx_generation_) {
        for (size_t i = 0; i < BLE_BUFF_SIZE; ++i) previous_[i] = driver_->send_data[i];
      }
      ~CommandStagingGuard() {
        if (generation_ == driver_->tx_generation_)
          for (size_t i = 0; i < BLE_BUFF_SIZE; ++i) driver_->send_data[i] = previous_[i];
      }
     private:
      R4SDriver *driver_;
      uint32_t generation_;
      std::array<uint8_t, BLE_BUFF_SIZE> previous_{};
    };
    bool queue_command_(uint8_t command);
    bool dequeue_command_(uint8_t &command) { return this->pending_.pop(command, this->send_data); }
    bool has_pending_commands_() const { return !this->pending_.empty(); }
    void write_command_(uint8_t command);
    bool valid_response_(const uint8_t *data, int size, size_t status_size) const {
      return size >= 0 && valid_response(data, static_cast<size_t>(size), status_size);
    }
    DrvState state_{DrvState::IDLE};
    R4SEngine *parent_{nullptr};
    CommandQueue pending_{};
    bool initialized_{false};
    uint32_t tx_generation_{0};
    uint32_t state_since_ms_{0};
    uint32_t command_sent_ms_{0};
    uint32_t last_clock_attempt_ms_{0};
    uint8_t auth_attempts_{0};
    bool waiting_response_{false};
    uint8_t expected_command_{0};
    uint8_t expected_sequence_{0};
    bool clock_synchronized_{false};
    int32_t sent_clock_offset_{0};
    int32_t synchronized_clock_offset_{0};
    bool needs_clock_sync_() const {
      return this->supports_device_clock_() && this->notify_data_time > 0 &&
             (!this->clock_synchronized_ || this->tz_offset != this->synchronized_clock_offset_) &&
             (this->last_clock_attempt_ms_ == 0 || millis() - this->last_clock_attempt_ms_ >= 60000);
    }
    virtual bool supports_device_clock_() const { return false; }
};

class R4SEngine : public Component {
  public:
    float get_setup_priority() const override;
    void setup() override;
    void dump_config() override;
    void loop() override;
    
    void set_scan_duration(uint32_t scan_duration) { scan_duration_ = scan_duration; }
    void set_scan_active(bool scan_active) { scan_params_.scan_type = scan_active ? BLE_SCAN_TYPE_ACTIVE:BLE_SCAN_TYPE_PASSIVE; }
    void set_scan_parameters(uint32_t scan_interval, uint32_t scan_window) {
      scan_params_.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
      scan_params_.scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL;
      scan_params_.scan_interval = scan_interval;
      scan_params_.scan_window = scan_window;
      scan_params_.scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE;
    }
    void set_time(time::RealTimeClock *time) { time_ = time; }
    time::RealTimeClock *get_time() const { return time_; }
    void set_monitor(bool scan_monitor) { scan_monitor_ = scan_monitor; }
    bool get_monitor() const { return scan_monitor_; }
    void register_r4s_driver(R4SDriver *driver) {
      driver->app_id = ++this->app_id_;
      driver->set_engine_parent(this);
      this->drivers_.push_back(driver);
    }
    std::string address_str(uint64_t addr) const;
    void start_scan();
    void finish_scan_();
    void request_scan_stop_();

  protected:
    static bool ble_setup_();
    static void gap_event_handler_(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void gattc_event_handler_(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
    void process_gap_event_(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    void process_gattc_event_(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
    void enqueue_event_(QueuedBLEEvent &event, bool advertisement);
    void recover_from_event_loss_();

    int app_id_{0};
    esp_ble_scan_params_t scan_params_{};
    uint32_t scan_duration_{30};
    bool is_scaned_ = false;  // Includes asynchronous scan startup.
    bool scan_stop_requested_{false};
    QueueHandle_t events_{nullptr};
    std::atomic<uint32_t> dropped_advertisements_{0};
    std::atomic<uint32_t> lost_events_{0};
    uint32_t last_loop_ms_{0};
    uint32_t uptime_seconds_{0};
    bool scan_monitor_ = false;
    
    std::vector<ScanDevice> new_scan_dev_;
    std::vector<R4SDriver *> drivers_;
    time::RealTimeClock *time_{nullptr};
};
// NOLINTNEXTLINE
extern R4SEngine *global_r4s_engine;

}  // namespace ready4sky
}  // namespace esphome

#endif


