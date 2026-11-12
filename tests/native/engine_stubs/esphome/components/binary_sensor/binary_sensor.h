#pragma once
namespace esphome::binary_sensor {
class BinarySensor {
 public:
  bool state{false};
  bool available{false};
  int publications{0};
  void publish_state(bool value) { state=value; available=true; ++publications; }
  void invalidate_state() { available=false; }
};
}
