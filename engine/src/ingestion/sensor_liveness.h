#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

namespace aurora::ingestion {

class SensorLivenessMonitor {
 public:
  explicit SensorLivenessMonitor(double timeout_sec);

  void observe(std::uint32_t sensor_id, double now_sec);

  [[nodiscard]] bool has_seen(
      std::uint32_t sensor_id) const;

  [[nodiscard]] bool is_timed_out(
      std::uint32_t sensor_id,
      double now_sec) const;

  [[nodiscard]] std::vector<std::uint32_t>
  timed_out_sensors(double now_sec) const;

 private:
  double timeout_sec_;
  std::unordered_map<std::uint32_t, double> last_seen_sec_;
};

}  // namespace aurora::ingestion