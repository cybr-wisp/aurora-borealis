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

  aurora::ingestion::SensorLivenessMonitor
      liveness(1.0);

  std::unordered_set<std::uint32_t>
      timed_out_sensors;

  std::uint64_t accepted = 0;
  std::uint64_t rejected = 0;
  std::uint64_t timeout_events = 0;
  std::uint64_t recovery_events = 0;

  receiver.start();

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::seconds(duration_sec);

  while (std::chrono::steady_clock::now() < deadline) {
    const double now =
        std::chrono::duration<double>(
            std::chrono::system_clock::now()
                .time_since_epoch())
            .count();

    if (auto observation = queue.pop()) {
      const auto result =
          validator.validate(*observation, now);

      if (result ==
          aurora::ingestion::ValidationResult::kAccept) {
        ++accepted;

        const auto sensor_id =
            observation->sensor_id();

        if (timed_out_sensors.erase(sensor_id) > 0) {
          ++recovery_events;

          std::cout
              << "sensor_recovered sensor_id="
              << sensor_id
              << "\n";
        }

        liveness.observe(sensor_id, now);
      } else {
        ++rejected;
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
      << " rejected=" << rejected
      << " malformed=" << counters.malformed
      << " queue_full=" << counters.queue_full
      << " sensor_timeouts=" << timeout_events
      << " sensor_recoveries=" << recovery_events
      << "\n";

  return EXIT_SUCCESS;
}