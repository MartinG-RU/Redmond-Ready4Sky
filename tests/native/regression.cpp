#include "../../components/skyplug/skyplug.h"
#include "../../components/skycoffee/skycoffee.h"
#include "../../components/skysmoke/skysmoke.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

uint32_t fake_millis = 100;
std::vector<std::vector<uint8_t>> writes;
int disconnects = 0;
int esp_ble_gattc_write_char(int, uint16_t, uint16_t, int len, const uint8_t *data, int, int) {
  writes.emplace_back(data, data + len);
  return 0;
}
int esp_ble_gap_disconnect(const uint8_t *) { ++disconnects; return 0; }
namespace esphome::ready4sky {
std::string R4SEngine::address_str(uint64_t) const { return "00:00:00:00:00:00"; }
void R4SEngine::start_scan() {}
}
class TestPlug : public esphome::skyplug::SkyPlug {
 public:
  using SkyPlug::sync_data_;
  using SkyPlug::parse_response_;
  using SkyPlug::device_offline_;
  void begin() { is_authorize = true; note_status_received_(); is_ready = true; }
  void notify_status() {
    uint8_t status[15]{};
    status[0] = 0x55; status[1] = cmd_count; status[2] = 0x06; status[14] = 0xAA;
    parse_response_(status, sizeof(status), 1);
  }
  void notify_ok(uint8_t command) {
    uint8_t ack[]{0x55, cmd_count, command, 0x01, 0xAA};
    parse_response_(ack, sizeof(ack), 1);
  }
};
class TestCoffee : public esphome::skycoffee::SkyCoffee {
 public:
  using SkyCoffee::parse_response_;
  using SkyCoffee::sync_data_;
  using SkyCoffee::device_offline_;
  void begin(uint8_t model_type) {
    set_type(model_type); setup(); is_authorize = true; note_status_received_(); is_ready = true;
  }
  void notify_status() {
    uint8_t status[18]{};
    status[0] = 0x55; status[1] = cmd_count; status[2] = 0x06;
    status[11] = 0; status[14] = 0; status[16] = 0; status[17] = 0xAA;
    parse_response_(status, sizeof(status), 1);
  }
  void notify_ok(uint8_t command) {
    uint8_t ack[]{0x55, cmd_count, command, 0x01, 0xAA};
    parse_response_(ack, sizeof(ack), 1);
  }
};
class TestSmoke : public esphome::skysmoke::SkySmoke {
 public:
  using SkySmoke::parse_response_;
  using SkySmoke::device_offline_;
};
int main() {
  using namespace esphome::ready4sky;
  // Pure protocol: oversize, truncation, markers, command-specific minimum.
  uint8_t short_frame[]{0x55, 1, 0x47, 0x00, 0xAA};
  assert(!valid_response(short_frame, sizeof(short_frame), 15));
  uint8_t good[18]{}; good[0]=0x55; good[1]=1; good[2]=0x47; good[17]=0xAA;
  assert(valid_response(good, sizeof(good), 15));
  assert(!valid_response(good, 25, 15));
  good[0]=0; assert(!valid_response(good, sizeof(good), 15));
  uint8_t le[]{0xff, 0xff, 0xff, 0xff};
  assert(read_le32(le)==0xffffffffu);
  CommandQueue q; uint8_t data[R4S_FRAME_SIZE]{};
  for (int i=0; i<8; ++i) { data[3]=i; assert(q.push(0x16,data)); }
  assert(!q.push(0x16,data));
  for (int i=0; i<8; ++i) { uint8_t command; assert(q.pop(command,data)); assert(data[3]==i); }
  assert(q.empty());

  esphome::switch_::Switch power;
  TestPlug plug; plug.set_power(&power); plug.begin();
  plug.sync_data_(); // actual status request writes AA into the shared buffer
  assert(writes.back()[2]==0x06 && writes.back()[3]==0xAA);
  plug.send_lock(true);             // queue 0x16 with 01
  plug.send_remember(false);        // queue 0x44 with 00
  assert(plug.send_data[3]==0xAA);  // active in-flight metadata restored
  plug.notify_status();
  plug.sync_data_();
  assert(writes.back().size()==5 && writes.back()[2]==0x16 && writes.back()[3]==0x01);
  plug.notify_ok(0x16);             // driver requests status after ack
  plug.notify_status();
  plug.sync_data_();
  assert(writes.back()[2]==0x44 && writes.back()[3]==0x00);
  // OFF supersedes old pending commands and cannot be rejected by a full queue.
  plug.send_lock(true);
  plug.send_power(false);
  plug.notify_ok(0x44);
  plug.notify_status();
  plug.sync_data_();
  assert(writes.back()[2]==0x04);
  const size_t count_before_timeout = writes.size();
  fake_millis += 16000;
  plug.check_timeout_(fake_millis);
  assert(disconnects==1);
  plug.send_lock(true);
  assert(writes.size()==count_before_timeout); // never replay while offline

  // Switch-level OFF must cancel queued ON even when the last confirmed state
  // is already OFF; comparing only the cached state would skip the request.
  TestPlug switched_plug; esphome::switch_::Switch switched_plug_power;
  switched_plug.set_power(&switched_plug_power); switched_plug.begin(); switched_plug.notify_status();
  switched_plug.sync_data_();
  esphome::skyplug::SkyPlugPowerSwitch plug_ui(&switched_plug);
  plug_ui.write_state(true); plug_ui.write_state(false);
  switched_plug.notify_status(); switched_plug.sync_data_();
  assert(writes.back()[2]==0x04);

  // Real coffee driver: snapshots retain the lock and strength arguments.
  esphome::switch_::Switch coffee_power, coffee_lock, coffee_strength;
  TestCoffee coffee; coffee.set_power(&coffee_power); coffee.set_lock(&coffee_lock);
  coffee.set_strength(&coffee_strength); coffee.begin(2); // RCM-M1519S type
  coffee.sync_data_();
  assert(writes.back()[2] == 0x06 && writes.back()[3] == 0xAA);
  coffee.send_lock(true);
  coffee.send_strength(true);
  assert(coffee.send_data[3] == 0xAA);
  coffee.notify_status(); coffee.sync_data_();
  assert(writes.back()[2] == 0x3E && writes.back()[3] == 1);
  coffee.notify_ok(0x3E); coffee.notify_status(); coffee.sync_data_();
  assert(writes.back()[2] == 0x1B && writes.back()[3] == 1);
  const auto before_bad_status = writes.size();
  uint8_t short_coffee[13]{};
  short_coffee[0] = 0x55; short_coffee[1] = coffee.cmd_count;
  short_coffee[2] = 0x06; short_coffee[12] = 0xAA;
  coffee.parse_response_(short_coffee, sizeof(short_coffee), 1);
  assert(writes.size() == before_bad_status && !coffee.is_ready);
  coffee.clear_pending_commands_(); coffee.device_offline_(); coffee.send_power(true);
  assert(writes.size() == before_bad_status);
  // Models with no parsed lock byte must not be blocked by its initial 0xFF.
  esphome::switch_::Switch old_coffee_power;
  TestCoffee old_coffee; old_coffee.set_power(&old_coffee_power); old_coffee.begin(1);
  uint8_t old_status[13]{}; old_status[0]=0x55; old_status[1]=1;
  old_status[2]=0x06; old_status[11]=0; old_status[12]=0xAA;
  old_coffee.parse_response_(old_status,sizeof(old_status),1);
  const auto before_start = writes.size();
  old_coffee.send_power(true);
  assert(writes.size()==before_start+1 && writes.back()[2]==0x03);
  // For known lock-capable models a locked status still blocks ON.
  TestCoffee locked_coffee; locked_coffee.set_power(&old_coffee_power); locked_coffee.begin(2);
  uint8_t locked_status[18]{}; locked_status[0]=0x55; locked_status[1]=1;
  locked_status[2]=0x06; locked_status[16]=1; locked_status[17]=0xAA;
  locked_coffee.parse_response_(locked_status,sizeof(locked_status),1);
  const auto before_blocked = writes.size();
  locked_coffee.send_power(true); assert(writes.size()==before_blocked);
  // Recheck the lock at dequeue: it may have changed after ON was queued.
  TestCoffee lock_race; lock_race.set_power(&old_coffee_power); lock_race.begin(2);
  lock_race.notify_status(); lock_race.sync_data_(); lock_race.send_power(true);
  locked_status[1]=lock_race.cmd_count;
  lock_race.parse_response_(locked_status,sizeof(locked_status),1);
  const auto before_locked_dequeue = writes.size();
  lock_race.sync_data_(); assert(writes.size()==before_locked_dequeue);
  // OFF through the switch must supersede an ON waiting behind a status poll.
  TestCoffee coffee_cancel; coffee_cancel.set_power(&old_coffee_power); coffee_cancel.begin(2);
  coffee_cancel.notify_status(); coffee_cancel.sync_data_();
  esphome::skycoffee::SkyCoffeePowerSwitch coffee_ui(&coffee_cancel);
  coffee_ui.write_state(true); coffee_ui.write_state(false);
  coffee_cancel.notify_status(); coffee_cancel.sync_data_();
  assert(writes.back()[2]==0x04);

  esphome::binary_sensor::BinarySensor smoke;
  esphome::sensor::Sensor temperature, battery;
  TestSmoke detector; detector.set_smoke(&smoke); detector.set_temperature(&temperature); detector.set_battery_level(&battery);
  detector.is_authorize=true;
  uint8_t status[14]{}; status[0]=0x55; status[1]=1; status[2]=0x06; status[4]=5; status[5]=0; status[6]=80; status[12]=1; status[13]=0xAA;
  detector.parse_response_(status,sizeof(status),1);
  assert(smoke.available && smoke.state && temperature.publications==1);
  status[5]=1; // low byte is unchanged: temperature still must update
  detector.parse_response_(status,sizeof(status),2);
  assert(temperature.publications==2);
  detector.device_offline_();
  assert(!smoke.available && std::isnan(temperature.state) && std::isnan(battery.state));
  detector.parse_response_(status,sizeof(status),3);
  assert(smoke.available && smoke.publications==2 && temperature.publications==4);
  const int n=temperature.publications;
  detector.parse_response_(status,3,4);
  assert(temperature.publications==n);
  std::puts("native regression checks passed");
}
