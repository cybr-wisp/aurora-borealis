#pragma once
#include "observation.pb.h"
#include "ingestion/observation_queue.h"
#include <atomic>
#include <cstdint>
#include <thread>

namespace aurora::ingestion {
struct ReceiverCounters { std::uint64_t received=0, malformed=0, queue_full=0; };
class UdpReceiver {
 public:
  UdpReceiver(std::uint16_t port, SpscRing<aurora::proto::Observation,4096>& queue);
  ~UdpReceiver();
  void start();
  void stop();
  ReceiverCounters counters() const;
 private:
  void run();
  std::uint16_t port_;
  SpscRing<aurora::proto::Observation,4096>& queue_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  std::atomic<std::uint64_t> received_{0}, malformed_{0}, queue_full_{0};
};
}
