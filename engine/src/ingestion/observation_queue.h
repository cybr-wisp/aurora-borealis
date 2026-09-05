#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <optional>
#include <utility>

namespace aurora::ingestion {

template <typename T, std::size_t Capacity>
class SpscRing {
  static_assert(Capacity >= 2);

 public:
  static constexpr std::size_t usable_capacity() noexcept {
    return Capacity - 1;
  }

  bool push(T value) {
    const auto head = head_.load(std::memory_order_relaxed);
    const auto next = (head + 1) % Capacity;
    const auto tail = tail_.load(std::memory_order_acquire);

    if (next == tail) {
      return false;
    }

    slots_[head] = std::move(value);
    head_.store(next, std::memory_order_release);

    const auto depth = distance(next, tail);
    auto observed = high_water_mark_.load(std::memory_order_relaxed);
    while (depth > observed &&
           !high_water_mark_.compare_exchange_weak(
               observed,
               depth,
               std::memory_order_relaxed,
               std::memory_order_relaxed)) {
    }

    return true;
  }

  std::optional<T> pop() {
    const auto tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
      return std::nullopt;
    }

    T value = std::move(slots_[tail]);
    tail_.store((tail + 1) % Capacity, std::memory_order_release);
    return value;
  }

  // Consumer-only backlog shedding. The producer never advances tail_, which
  // preserves the SPSC ownership rule while allowing the consumer to discard
  // stale work during overload.
  std::size_t discard_oldest(std::size_t max_count) {
    std::size_t discarded = 0;
    while (discarded < max_count) {
      if (!pop()) {
        break;
      }
      ++discarded;
    }
    return discarded;
  }

  [[nodiscard]] std::size_t size_approx() const noexcept {
    const auto head = head_.load(std::memory_order_acquire);
    const auto tail = tail_.load(std::memory_order_acquire);
    return distance(head, tail);
  }

  [[nodiscard]] std::size_t high_water_mark() const noexcept {
    return high_water_mark_.load(std::memory_order_relaxed);
  }

 private:
  static constexpr std::size_t distance(
      std::size_t head,
      std::size_t tail) noexcept {
    return head >= tail ? head - tail : Capacity - tail + head;
  }

  std::array<T, Capacity> slots_{};
  alignas(64) std::atomic<std::size_t> head_{0};
  alignas(64) std::atomic<std::size_t> tail_{0};
  std::atomic<std::size_t> high_water_mark_{0};
};

}  // namespace aurora::ingestion
