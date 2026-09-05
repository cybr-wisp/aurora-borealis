#pragma once

#include "association/track_manager.h"
#include "estimation/adaptive_noise.h"
#include "estimation/measurement_model.h"
#include "estimation/process_model.h"

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace aurora::pipeline {

struct TrackerConfig {
  double accel_spectral_density{4.0};
  double gate_threshold{7.814727903251179};

  int confirmation_hits{3};
  int coast_after_misses{1};
  int tentative_delete_misses{2};
  int delete_after_misses{5};

  double initial_position_variance_m2{400.0};
  double initial_velocity_variance_m2ps2{2500.0};

  bool enable_adaptive_noise{false};
  estimation::AdaptiveNoiseConfig adaptive_noise{};
};

struct SensorMeasurement {
  std::uint32_t sensor_id{0};
  Eigen::Vector3d sensor_position_enu_m =
      Eigen::Vector3d::Zero();
  estimation::SphericalMeasurement measurement{};
};

struct FrameAssociation {
  std::uint64_t track_id{0};
  std::size_t measurement_index{0};
  std::uint32_t sensor_id{0};
  double mahalanobis_sq{0.0};
};

struct SensorNoiseSnapshot {
  std::uint32_t sensor_id{0};
  estimation::MeasurementCovariance estimated_r =
      estimation::MeasurementCovariance::Identity();
  std::size_t sample_count{0};
  bool ready{false};
  bool degraded{false};
  double max_variance_ratio{1.0};
};

struct FrameResult {
  std::vector<FrameAssociation> assignments;
  std::vector<std::uint64_t> created_tracks;
  std::vector<std::size_t> unassigned_measurements;
  std::vector<SensorNoiseSnapshot> sensor_noise;
  std::size_t gated_pair_count{0};
};

class MultiObjectTracker {
 public:
  explicit MultiObjectTracker(TrackerConfig config = {});

  FrameResult process_frame(
      const std::vector<estimation::SphericalMeasurement>& measurements,
      const Eigen::Vector3d& sensor_position_enu_m,
      double dt_sec);

  FrameResult process_sensor_frame(
      const std::vector<SensorMeasurement>& measurements,
      double dt_sec);

  [[nodiscard]]
  const std::vector<association::Track>& tracks() const noexcept {
    return tracks_.tracks();
  }

  [[nodiscard]]
  estimation::AdaptiveNoiseStatus sensor_noise_status(
      std::uint32_t sensor_id,
      const estimation::MeasurementCovariance& configured_r) const {
    return adaptive_noise_.status(sensor_id, configured_r);
  }

 private:
  estimation::StateVector initialize_state(
      const estimation::SphericalMeasurement& measurement,
      const Eigen::Vector3d& sensor_position_enu_m) const;

  estimation::Covariance initialize_covariance() const;

  TrackerConfig config_;
  association::TrackManager tracks_;
  estimation::ConstantVelocityModel process_;
  estimation::PerSensorAdaptiveNoise adaptive_noise_;
};

}  // namespace aurora::pipeline
