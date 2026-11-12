#pragma once
#include <cstddef>
#include <cstring>
#include <deque>
#include <vector>
struct MockQueue { size_t capacity, width; std::deque<std::vector<unsigned char>> items; };
using QueueHandle_t = MockQueue *;
inline QueueHandle_t xQueueCreate(size_t n, size_t width) { return new MockQueue{n, width, {}}; }
inline unsigned uxQueueSpacesAvailable(QueueHandle_t q) { return q->capacity-q->items.size(); }
inline int xQueueSend(QueueHandle_t q, const void *item, int) {
  if(q->items.size()==q->capacity) return 0;
  auto *b=static_cast<const unsigned char *>(item);
  q->items.emplace_back(b,b+q->width); return 1;
}
inline int xQueueReceive(QueueHandle_t q, void *item, int) {
  if(q->items.empty()) return 0;
  std::memcpy(item,q->items.front().data(),q->width); q->items.pop_front(); return 1;
}
inline void xQueueReset(QueueHandle_t q) { q->items.clear(); }
