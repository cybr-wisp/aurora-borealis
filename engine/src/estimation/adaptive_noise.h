#pragma once

#include "estimation/state.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>

namespace aurora::estimation {

struct AdaptiveNoiseConfig {
  std::size_t window_size{50};
  std::size_t min_samples{15};
  double alpha{0.05};
  double min_scale{0.25};
  double max_scale{25.0};
  double degradation_ratio{2.0};
};

struct AdaptiveNoiseStatus {
  MeasurementCovariance estimated_r =
      MeasurementCovariance::Identity();
  std::size_t sample_count{0};
  bool ready{false};
  bool degraded{false};
  double max_variance_ratio{1.0};
};

class AdaptiveMeasurementNoise {
 public:
  explicit AdaptiveMeasurementNoise(AdaptiveNoiseConfig config = {});

  void observe(
      const MeasurementVector& innovation,
      const MeasurementCovariance& predicted_measurement_covariance,
      const MeasurementCovariance& configured_r);

  [[nodiscard]]
  MeasurementCovariance effective_r(
      const MeasurementCovariance& configured_r) const;

  [[nodiscard]]
  AdaptiveNoiseStatus status(
      const MeasurementCovariance& configured_r) const;

  void reset();

 private:
  struct Sample {
    std::array<double, 3> innovation_sq{};
    std::array<double, 3> predicted_variance{};
  };

  void validate_configured_r(
      const MeasurementCovariance& configured_r) const;

  AdaptiveNoiseConfig config_;
  std::deque<Sample> window_;
  MeasurementCovariance current_r_ =
      MeasurementCovariance::Identity();
  bool initialized_{false};
};

class PerSensorAdaptiveNoise {
 public:
  explicit PerSensorAdaptiveNoise(AdaptiveNoiseConfig config = {});

  [[nodiscard]]
  MeasurementCovariance effective_r(
      std::uint32_t sensor_id,
      const MeasurementCovariance& configured_r) const;

  void observe(
      std::uint32_t sensor_id,
      const MeasurementVector& innovation,
      const MeasurementCovariance& predicted_measurement_covariance,
      const MeasurementCovariance& configured_r);

  [[nodiscard]]
  AdaptiveNoiseStatus status(
      std::uint32_t sensor_id,
      const MeasurementCovariance& configured_r) const;

  void reset_sensor(std::uint32_t sensor_id);
  void reset_all();

 private:
  AdaptiveNoiseConfig config_;
  std::unordered_map<std::uint32_t, AdaptiveMeasurementNoise> estimators_;
};

}  // namespace aurora::estimation
