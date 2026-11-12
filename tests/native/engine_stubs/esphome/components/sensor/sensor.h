#pragma once
#include <cmath>
namespace esphome::sensor {
class Sensor {
 public:
  float state{NAN};
  int publications{0};
  void publish_state(float value) { state=value; ++publications; }
};
}
