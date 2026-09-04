#include "ingestion/sensor_liveness.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aurora::ingestion {

SensorLivenessMonitor::SensorLivenessMonitor(
    double timeout_sec)
    : timeout_sec_(timeout_sec) {
  if (!std::isfinite(timeout_sec_) ||
      timeout_sec_ <= 0.0) {
    throw std::invalid_argument(
        "sensor timeout must be finite and positive");
  }
}

void SensorLivenessMonitor::observe(
    std::uint32_t sensor_id,
    double now_sec) {
  if (!std::isfinite(now_sec)) {
    throw std::invalid_argument(
        "observation time must be finite");
  }

  last_seen_sec_[sensor_id] = now_sec;
}

bool SensorLivenessMonitor::has_seen(
    std::uint32_t sensor_id) const {
  return last_seen_sec_.contains(sensor_id);
}

bool SensorLivenessMonitor::is_timed_out(
    std::uint32_t sensor_id,
    double now_sec) const {
  const auto it = last_seen_sec_.find(sensor_id);

  if (it == last_seen_sec_.end()) {
    return false;
  }

  if (!std::isfinite(now_sec)) {
    throw std::invalid_argument(
        "current time must be finite");
  }

  return now_sec - it->second > timeout_sec_;
}

std::vector<std::uint32_t>
SensorLivenessMonitor::timed_out_sensors(
    double now_sec) const {
  std::vector<std::uint32_t> result;

  for (const auto& [sensor_id, last_seen] :
       last_seen_sec_) {
    if (now_sec - last_seen > timeout_sec_) {
      result.push_back(sensor_id);
    }
  }

  std::sort(result.begin(), result.end());

  return result;
}

}  // namespace aurora::ingestion