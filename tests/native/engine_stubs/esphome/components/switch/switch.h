#pragma once
namespace esphome::switch_ {
class Switch {
 public:
  bool state{false};
  virtual ~Switch() = default;
  virtual void write_state(bool) {}
  void publish_state(bool value) { state=value; }
};
}
