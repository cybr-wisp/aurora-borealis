#include "ingestion/observation_queue.h"
#include "ingestion/sensor_liveness.h"
#include "ingestion/udp_receiver.h"
#include "ingestion/validator.h"
#include "observation.pb.h"

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <unordered_set>

int main(int argc, char** argv) {
  const std::uint16_t port =
      argc > 1
          ? static_cast<std::uint16_t>(std::stoi(argv[1]))
          : 46000;

  const int duration_sec =
      argc > 2 ? std::stoi(argv[2]) : 10;

  aurora::ingestion::SpscRing<
      aurora::proto::Observation,
      4096>
      queue;

  aurora::ingestion::UdpReceiver receiver(
      port,
      queue);

  aurora::ingestion::Validator validator(1.0);
  aurora::ingestion::SensorLivenessMonitor liveness(1.0);

  std::unordered_set<std::uint32_t> timed_out_sensors;

  std::uint64_t accepted = 0;
  std::uint64_t rejected_duplicate = 0;
  std::uint64_t rejected_stale = 0;
  std::uint64_t rejected_invalid = 0;
  std::uint64_t overload_shed_oldest = 0;
  std::uint64_t timeout_events = 0;
  std::uint64_t recovery_events = 0;

  // A real-time tracker should not spend seconds draining stale work after an
  // overload burst. Only the consumer advances tail_, so shedding here keeps
  // the ring lock-free SPSC while preserving freshness.
  constexpr std::size_t kShedThreshold = 1024;
  constexpr std::size_t kTargetDepthAfterShed = 256;

  receiver.start();

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::seconds(duration_sec);

  while (std::chrono::steady_clock::now() < deadline) {
    const auto depth = queue.size_approx();
    if (depth > kShedThreshold) {
      overload_shed_oldest += queue.discard_oldest(
          depth - kTargetDepthAfterShed);
    }

    const double now =
        std::chrono::duration<double>(
            std::chrono::system_clock::now()
                .time_since_epoch())
            .count();

    if (auto observation = queue.pop()) {
      const auto result =
          validator.validate(*observation, now);

      switch (result) {
        case aurora::ingestion::ValidationResult::kAccept: {
          ++accepted;

          const auto sensor_id = observation->sensor_id();

          if (timed_out_sensors.erase(sensor_id) > 0) {
            ++recovery_events;
            std::cout
                << "sensor_recovered sensor_id="
                << sensor_id
                << "\n";
          }

          liveness.observe(sensor_id, now);
          break;
        }

        case aurora::ingestion::ValidationResult::kDuplicateOrOldSequence:
          ++rejected_duplicate;
          break;

        case aurora::ingestion::ValidationResult::kStale:
          ++rejected_stale;
          break;

        case aurora::ingestion::ValidationResult::kInvalid:
          ++rejected_invalid;
          break;
      }
    } else {
      std::this_thread::sleep_for(
          std::chrono::microseconds(50));
    }

    for (const auto sensor_id :
         liveness.timed_out_sensors(now)) {
      if (timed_out_sensors.insert(sensor_id).second) {
        ++timeout_events;

        std::cout
            << "sensor_timeout sensor_id="
            << sensor_id
            << "\n";
      }
    }
  }

  receiver.stop();

  const auto counters = receiver.counters();

  std::cout
      << "received=" << counters.received
      << " accepted=" << accepted
      << " rejected_duplicate=" << rejected_duplicate
      << " rejected_stale=" << rejected_stale
      << " rejected_invalid=" << rejected_invalid
      << " malformed=" << counters.malformed
      << " queue_full=" << counters.queue_full
      << " queue_high_water=" << queue.high_water_mark()
      << " shed_oldest=" << overload_shed_oldest
      << " sensor_timeouts=" << timeout_events
      << " sensor_recoveries=" << recovery_events
      << "\n";

  return EXIT_SUCCESS;
}

