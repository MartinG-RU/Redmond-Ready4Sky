#pragma once
#include <cstdint>
namespace esphome {
struct ESPTime { int32_t timestamp=1700000000; bool is_valid() const { return true; } static int32_t timezone_offset() { return 0; } };
}
