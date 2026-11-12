#pragma once
#include "esphome/core/time.h"
namespace esphome::time { class RealTimeClock { public: ESPTime now() const { return {}; } }; }
