// The real BLE engine with an intentionally simplified, single-threaded IDF
// simulator. This does not model radio timing or replace tests on an ESP32.
#include "../../components/ready4sky/ready4sky.h"
#include <cassert>
#include <cstring>
#include <cstdio>

uint32_t fake_millis=100;
int fake_stop_calls=0, fake_open_calls=0, fake_disconnect_calls=0, fake_close_calls=0;
int fake_search_calls=0, fake_write_calls=0, fake_cancel_calls=0;
using namespace esphome::ready4sky;

class FakeDriver : public R4SDriver {
 public:
  int offlines=0, onlines=0, parses=0, holds=0, polls=0;
  uint8_t last_value=0;
  void parse_response_(uint8_t *data, int8_t, uint32_t) override { ++parses; last_value=data[3]; }
  void device_online_() override { ++onlines; }
  void device_offline_() override { ++offlines; is_authorize=false; }
  void sync_data_() override { ++polls; }
  void verify_contig_() override { conf_error=false; }
  size_t status_frame_minimum_() const override { return 15; }
  void hold_if_waiting_() override { ++holds; }
  void issue(uint8_t cmd, uint8_t seq) {
    send_data[0]=0x55; send_data[1]=seq; send_data[2]=cmd; send_data[3]=1; send_data[4]=0xAA;
    send_data_len=5; write_command_(cmd);
  }
  bool waiting() const { return waiting_response_; }
};

class FakeEngine : public R4SEngine {
 public:
  esphome::time::RealTimeClock clock;
  FakeEngine() { global_r4s_engine=this; events_=xQueueCreate(48,sizeof(QueuedBLEEvent)); set_time(&clock); }
  ~FakeEngine() { delete events_; global_r4s_engine=nullptr; }
  void gap(int event, esp_ble_gap_cb_param_t &p) { gap_event_handler_(event,&p); }
  void gatt(int event, int iface, esp_ble_gattc_cb_param_t &p) { gattc_event_handler_(event,iface,&p); }
  bool scanning() const { return is_scaned_; }
  bool stopping() const { return scan_stop_requested_; }
  size_t observed() const { return new_scan_dev_.size(); }
};

static void address(uint8_t *to, int n) { uint8_t b[6]={0xAA,0xBB,0xCC,0xDD,0xEE,uint8_t(n)}; std::memcpy(to,b,6); }
static void adv(FakeEngine &e, int n) {
  esp_ble_gap_cb_param_t p{}; p.scan_rst.search_evt=ESP_GAP_SEARCH_INQ_RES_EVT;
  address(p.scan_rst.bda,n); p.scan_rst.rssi=-44; p.scan_rst.ble_adv[0]=7;
  std::memcpy(p.scan_rst.ble_adv+1,"RCM-ONE",7);
  e.gap(ESP_GAP_BLE_SCAN_RESULT_EVT,p);
}
static void stop_complete(FakeEngine &e, int status=0) {
  esp_ble_gap_cb_param_t p{}; p.scan_stop_cmpl.status=status;
  e.gap(ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT,p); e.loop();
}
static void open_success(FakeEngine &e, int iface, int device, int conn) {
  esp_ble_gattc_cb_param_t p{}; address(p.open.remote_bda,device); p.open.status=0; p.open.conn_id=conn;
  e.gatt(ESP_GATTC_OPEN_EVT,iface,p); e.loop();
}

int main() {
  FakeEngine engine;
  FakeDriver first, second;
  first.address=0xAABBCCDDEE01ULL; first.gattc_if=1; first.usr_model="first";
  second.address=0xAABBCCDDEE02ULL; second.gattc_if=2; second.usr_model="second";
  engine.register_r4s_driver(&first); engine.register_r4s_driver(&second);
  engine.set_monitor(true);
  engine.start_scan(); assert(engine.scanning());
  // A busy radio environment must not keep stopping the scan before the
  // configured device appears, and the diagnostic list is bounded.
  for (int n=3; n<80; ++n) { adv(engine,n); engine.loop(); }
  assert(engine.scanning() && !engine.stopping() && engine.observed()==64);
  adv(engine,1); engine.loop();
  assert(first.state()==DrvState::DISCOVERED && engine.stopping() && fake_stop_calls==1);
  // A second advertisement arriving while stopping must not strand another
  // driver in DISCOVERED or initiate simultaneous GATT opens.
  adv(engine,2); engine.loop(); assert(second.state()==DrvState::IDLE);
  // Failed stop completion requests a retry rather than pretending it stopped.
  stop_complete(engine,1); assert(engine.scanning() && !engine.stopping() && fake_open_calls==0);
  assert(engine.fire_timeout("scan-stop-retry")); assert(engine.stopping() && fake_stop_calls==2);
  stop_complete(engine); assert(first.state()==DrvState::CONNECTING && fake_open_calls==1);
  open_success(engine,1,1,7); assert(first.state()==DrvState::CONNECTED && first.conn_id==7);

  esp_ble_gattc_cb_param_t p{};
  p.cfg_mtu.conn_id=99; engine.gatt(ESP_GATTC_CFG_MTU_EVT,1,p); engine.loop();
  assert(fake_search_calls==0); // stale connection is ignored
  p.cfg_mtu.conn_id=7; engine.gatt(ESP_GATTC_CFG_MTU_EVT,1,p); engine.loop();
  assert(fake_search_calls==1);
  p={}; p.search_res.conn_id=7; p.search_res.start_handle=1; p.search_res.end_handle=20;
  engine.gatt(ESP_GATTC_SEARCH_RES_EVT,1,p); engine.loop();
  p={}; p.search_cmpl.conn_id=7; engine.gatt(ESP_GATTC_SEARCH_CMPL_EVT,1,p); engine.loop();
  assert(first.tx_char_handle==10 && first.rx_char_handle==11);
  p={}; p.reg_for_notify.handle=11; engine.gatt(ESP_GATTC_REG_FOR_NOTIFY_EVT,1,p); engine.loop();
  p={}; p.write.conn_id=7; engine.gatt(ESP_GATTC_WRITE_DESCR_EVT,1,p); engine.loop();
  assert(first.onlines==1);
  engine.gatt(ESP_GATTC_WRITE_CHAR_EVT,1,p); engine.loop();
  assert(first.state()==DrvState::ESTABLISHED && engine.scanning());

  // A late failed OPEN or a disconnect for the wrong connection cannot take
  // an established session offline.
  p={}; p.open.status=1; p.open.conn_id=2; address(p.open.remote_bda,1);
  engine.gatt(ESP_GATTC_OPEN_EVT,1,p); engine.loop();
  p={}; p.disconnect.conn_id=99; address(p.disconnect.remote_bda,1);
  engine.gatt(ESP_GATTC_DISCONNECT_EVT,1,p); engine.loop();
  assert(first.state()==DrvState::ESTABLISHED && first.offlines==0);
  // A stale successful OPEN for an idle/discovered driver is actively closed,
  // including connection-id zero, instead of being mistaken for a live link.
  second.set_state(DrvState::DISCOVERED);
  address(second.remote_bda,2);
  p={}; p.open.status=0; p.open.conn_id=0; address(p.open.remote_bda,2);
  engine.gatt(ESP_GATTC_OPEN_EVT,2,p); engine.loop();
  assert(second.state()==DrvState::DISCOVERED && fake_close_calls==1);
  second.set_state(DrvState::IDLE);

  first.is_authorize=true; first.issue(0x16,42); assert(first.waiting());
  uint8_t status[15]{}; status[0]=0x55; status[1]=77; status[2]=0x06; status[3]=0x5A; status[14]=0xAA;
  p={}; p.notify.conn_id=7; p.notify.handle=11; p.notify.value_len=sizeof(status);
  p.notify.value=status; p.notify.is_notify=true; address(p.notify.remote_bda,1);
  engine.gatt(ESP_GATTC_NOTIFY_EVT,1,p);
  status[3]=0x11; // callback data must have been copied before returning
  engine.loop();
  assert(first.parses==1 && first.last_value==0x5A && first.holds==1 && first.waiting());
  uint8_t ack[]={0x55,42,0x16,0x01,0xAA}; p.notify.value=ack; p.notify.value_len=sizeof(ack);
  engine.gatt(ESP_GATTC_NOTIFY_EVT,1,p); engine.loop();
  assert(first.parses==2 && !first.waiting());

  // If the queue drops a critical event, fail safe: clear it, mark devices
  // offline, disconnect, and schedule rediscovery.
  p={}; p.write.conn_id=7;
  for(int i=0;i<49;i++) engine.gatt(ESP_GATTC_WRITE_CHAR_EVT,1,p);
  engine.loop();
  assert(first.offlines==1 && first.state()==DrvState::IDLE && fake_disconnect_calls>=1);
  assert(engine.timers.count("scan-retry")!=0);
  // If the stack never delivers DISCONNECT, the shutdown state is bounded and
  // discovery is retried; commands remain unavailable during the shutdown.
  first.set_state(DrvState::ESTABLISHED); first.is_authorize=true; first.issue(0x16,43);
  fake_millis += 15000; engine.loop();
  assert(first.state()==DrvState::DISCONNECTING && first.offlines==2 && !first.is_authorize);
  fake_millis += 10000; engine.loop();
  assert(first.state()==DrvState::IDLE && engine.timers.count("scan-retry")!=0);
  p={}; p.disconnect.conn_id=7; address(p.disconnect.remote_bda,1);
  engine.gatt(ESP_GATTC_DISCONNECT_EVT,1,p); engine.loop();
  assert(first.offlines==2);  // late event cannot invalidate a newer lifecycle
  std::puts("BLE engine simulated regression checks passed");
}
