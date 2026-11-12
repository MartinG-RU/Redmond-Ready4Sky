#pragma once
namespace esphome {
namespace setup_priority { constexpr float BLUETOOTH = 300; constexpr float AFTER_BLUETOOTH = 299; }
class Component {
 public:
  virtual ~Component() = default;
  virtual float get_setup_priority() const { return 0; }
  virtual void setup() {}
  virtual void dump_config() {}
  virtual void loop() {}
  void mark_failed() {}
};
}
