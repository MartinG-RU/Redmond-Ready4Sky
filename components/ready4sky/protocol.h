#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace esphome::ready4sky {

// The Ready4Sky frames used by these drivers fit into the ATT payload buffer.
static constexpr size_t R4S_FRAME_SIZE = 24;
static constexpr size_t R4S_PENDING_COMMANDS = 8;

inline uint32_t read_le32(const uint8_t *data) {
  return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

struct PendingCommand {
  uint8_t command{0};
  std::array<uint8_t, R4S_FRAME_SIZE> data{};
};

// Used only on the ESPHome loop thread. Full payloads (including driver metadata)
// are copied so a later status request cannot overwrite a pending command.
class CommandQueue {
 public:
  bool push(uint8_t command, const uint8_t *data) {
    if (this->size_ == this->items_.size()) return false;
    auto &item = this->items_[(this->head_ + this->size_) % this->items_.size()];
    item.command = command;
    for (size_t i = 0; i < R4S_FRAME_SIZE; ++i) item.data[i] = data[i];
    ++this->size_;
    return true;
  }
  bool pop(uint8_t &command, uint8_t *data) {
    if (this->size_ == 0) return false;
    const auto &item = this->items_[this->head_];
    command = item.command;
    for (size_t i = 0; i < R4S_FRAME_SIZE; ++i) data[i] = item.data[i];
    this->head_ = (this->head_ + 1) % this->items_.size();
    --this->size_;
    return true;
  }
  void clear() { this->head_ = this->size_ = 0; }
  bool empty() const { return this->size_ == 0; }
  size_t size() const { return this->size_; }

 private:
  std::array<PendingCommand, R4S_PENDING_COMMANDS> items_{};
  size_t head_{0};
  size_t size_{0};
};

// status_size includes the final AA byte. Other minimums correspond to the
// highest indexed byte in the existing response parsers, plus the end marker.
inline bool valid_response(const uint8_t *data, size_t size, size_t status_size) {
  if (data == nullptr || size < 5 || size > R4S_FRAME_SIZE || data[0] != 0x55 || data[size - 1] != 0xAA)
    return false;
  size_t required = 5;
  switch (data[2]) {
    case 0x01: required = 6; break;  // firmware major + minor
    case 0x06: required = status_size; break;
    case 0x33: required = 20; break; // light parameters
    case 0x35: required = 7; break;
    case 0x47: required = 18; break; // three 32-bit counters
    case 0x50: required = 11; break;
    default: break;
  }
  return size >= required;
}

}  // namespace esphome::ready4sky
