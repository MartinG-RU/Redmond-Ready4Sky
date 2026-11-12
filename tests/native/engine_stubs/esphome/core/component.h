#pragma once
#include <functional>
#include <map>
#include <string>
#include <cstdint>
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
  void set_timeout(const std::string &name, uint32_t, std::function<void()> fn) { timers[name]=std::move(fn); }
  void cancel_timeout(const std::string &name) { timers.erase(name); }
  bool fire_timeout(const std::string &name) {
    auto it=timers.find(name); if(it==timers.end()) return false;
    auto fn=std::move(it->second); timers.erase(it); fn(); return true;
  }
  std::map<std::string, std::function<void()>> timers;
};
}
