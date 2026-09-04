#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <optional>

namespace aurora::ingestion {
template <typename T, std::size_t Capacity>
class SpscRing {
  static_assert(Capacity >= 2);
 public:
  bool push(T value) {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto next = (head + 1) % Capacity;
    if (next == tail_.load(std::memory_order_acquire)) return false;
    slots_[head] = std::move(value);
    head_.store(next, std::memory_order_release);
    return true;
  }
  std::optional<T> pop() {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) return std::nullopt;
    T value = std::move(slots_[tail]);
    tail_.store((tail + 1) % Capacity, std::memory_order_release);
    return value;
  }
 private:
  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
};
}
