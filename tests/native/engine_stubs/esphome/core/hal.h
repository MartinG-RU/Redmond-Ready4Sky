#pragma once
#include <cstdint>
extern uint32_t fake_millis;
namespace esphome { inline uint32_t millis() { return fake_millis; } inline void delay(uint32_t) {} }
