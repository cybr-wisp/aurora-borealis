#include "ingestion/observation_queue.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct Item {
  std::uint64_t sequence{0};
  Clock::time_point produced_at{};
};

double percentile(
    const std::vector<double>& sorted,
    double p) {
  if (sorted.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const auto index = static_cast<std::size_t>(
      p * static_cast<double>(sorted.size() - 1));
  return sorted[index];
}

void busy_wait_us(int delay_us) {
  const auto deadline =
      Clock::now() + std::chrono::microseconds(delay_us);
  while (Clock::now() < deadline) {
  }
}

}  // namespace

int main() {
  constexpr std::uint64_t kItems = 50'000;
  constexpr std::size_t kShedThreshold = 1024;
  constexpr std::size_t kTargetDepth = 256;
  constexpr int kConsumerWorkUs = 200;

  aurora::ingestion::SpscRing<Item, 4096> queue;

  std::atomic<bool> producer_done{false};
  std::atomic<std::uint64_t> queue_full{0};

  std::uint64_t processed = 0;
  std::uint64_t shed_oldest = 0;
  std::vector<double> age_us;
  age_us.reserve(static_cast<std::size_t>(kItems));

  const auto started = Clock::now();

  std::thread producer([&] {
    for (std::uint64_t i = 0; i < kItems; ++i) {
      if (!queue.push(Item{i, Clock::now()})) {
        ++queue_full;
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  while (!producer_done.load(std::memory_order_acquire) ||
         queue.size_approx() > 0) {
    const auto depth = queue.size_approx();

    if (depth > kShedThreshold) {
      shed_oldest += queue.discard_oldest(
          depth - kTargetDepth);
    }

    if (auto item = queue.pop()) {
      const auto now = Clock::now();
      age_us.push_back(
          std::chrono::duration<double, std::micro>(
              now - item->produced_at)
              .count());

      ++processed;
      busy_wait_us(kConsumerWorkUs);
    } else {
      std::this_thread::yield();
    }
  }

  producer.join();

  std::sort(age_us.begin(), age_us.end());

  const auto elapsed =
      std::chrono::duration<double>(Clock::now() - started).count();

  std::cout << std::fixed << std::setprecision(3)
            << "produced=" << kItems
            << " processed=" << processed
            << " producer_queue_full=" << queue_full.load()
            << " consumer_shed_oldest=" << shed_oldest
            << " queue_high_water=" << queue.high_water_mark()
            << " elapsed_sec=" << elapsed
            << " p50_age_us=" << percentile(age_us, 0.50)
            << " p95_age_us=" << percentile(age_us, 0.95)
            << " p99_age_us=" << percentile(age_us, 0.99)
            << "\n";

  return 0;
}

