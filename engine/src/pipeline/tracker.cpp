#include "pipeline/tracker.h"

#include "association/gating.h"
#include "association/hungarian.h"
#include "estimation/ekf.h"
#include "geometry/conversions.h"

#include <Eigen/Dense>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace aurora::pipeline {

MultiObjectTracker::MultiObjectTracker(
    TrackerConfig config)
    : config_(config),
      tracks_(association::TrackManagerConfig{
          config.confirmation_hits,
          config.coast_after_misses,
          config.tentative_delete_misses,
          config.delete_after_misses}),
      process_(config.accel_spectral_density) {
  if (!(config_.gate_threshold > 0.0) ||
      !(config_.initial_position_variance_m2 > 0.0) ||
      !(config_.initial_velocity_variance_m2ps2 > 0.0)) {
    throw std::invalid_argument(
        "invalid multi-object tracker configuration");
  }
}

FrameResult MultiObjectTracker::process_frame(
    const std::vector<
        estimation::SphericalMeasurement>& measurements,
    const Eigen::Vector3d& sensor_position_enu_m,
    double dt_sec) {
  if (!(dt_sec > 0.0)) {
    throw std::invalid_argument(
        "dt_sec must be positive");
  }

  FrameResult result;

  const auto f = process_.transition(dt_sec);
  const auto q = process_.process_noise(dt_sec);
  const auto track_ids = tracks_.active_track_ids();

  // Time-align all active tracks to this measurement frame.
  for (const auto track_id : track_ids) {
    auto* track = tracks_.find(track_id);
    if (track == nullptr) {
      continue;
    }

    const auto predicted_state = f * track->state;
    estimation::Covariance predicted_covariance =
        f * track->covariance * f.transpose() + q;
    predicted_covariance =
        0.5 *
        (predicted_covariance +
         predicted_covariance.transpose());

    tracks_.update_estimate(
        track_id,
        predicted_state,
        predicted_covariance);
  }

  const estimation::RadarMeasurementModel model(
      sensor_position_enu_m);

  Eigen::MatrixXd costs(
      static_cast<Eigen::Index>(track_ids.size()),
      static_cast<Eigen::Index>(measurements.size()));

  costs.setConstant(config_.gate_threshold + 1.0);

  for (std::size_t row = 0;
       row < track_ids.size();
       ++row) {
    const auto* track = tracks_.find(track_ids[row]);
    if (track == nullptr) {
      continue;
    }

    for (std::size_t col = 0;
         col < measurements.size();
         ++col) {
      const auto gate = association::gate_measurement(
          track->state,
          track->covariance,
          measurements[col],
          model,
          config_.gate_threshold);

      if (gate.accepted) {
        costs(
            static_cast<Eigen::Index>(row),
            static_cast<Eigen::Index>(col)) =
            gate.mahalanobis_sq;
        ++result.gated_pair_count;
      }
    }
  }

  const auto assignment =
      association::hungarian_assign(
          costs,
          config_.gate_threshold);

  std::unordered_set<std::size_t>
      assigned_measurements;

  for (std::size_t row = 0;
       row < track_ids.size();
       ++row) {
    const int assigned_col =
        assignment.empty()
            ? -1
            : assignment[row];

    if (assigned_col < 0) {
      tracks_.mark_missed(track_ids[row]);
      continue;
    }

    const auto col =
        static_cast<std::size_t>(assigned_col);

    auto* track = tracks_.find(track_ids[row]);
    if (track == nullptr) {
      continue;
    }

    estimation::Ekf ekf(
        track->state,
        track->covariance,
        process_);

    ekf.update(measurements[col], model);

    tracks_.mark_associated(
        track_ids[row],
        ekf.state(),
        ekf.covariance());

    assigned_measurements.insert(col);

    result.assignments.push_back({
        track_ids[row],
        col,
        costs(
            static_cast<Eigen::Index>(row),
            static_cast<Eigen::Index>(col))});
  }

  for (std::size_t col = 0;
       col < measurements.size();
       ++col) {
    if (assigned_measurements.contains(col)) {
      continue;
    }

    const auto track_id = tracks_.create(
        initialize_state(
            measurements[col],
            sensor_position_enu_m),
        initialize_covariance());

    result.created_tracks.push_back(track_id);
    result.unassigned_measurements.push_back(col);
  }

  tracks_.prune_deleted();
  return result;
}

estimation::StateVector
MultiObjectTracker::initialize_state(
    const estimation::SphericalMeasurement& measurement,
    const Eigen::Vector3d& sensor_position_enu_m) const {
  estimation::StateVector state =
      estimation::StateVector::Zero();

  state.head<3>() =
      sensor_position_enu_m +
      geometry::spherical_to_cartesian(
          measurement.z(0),
          measurement.z(1),
          measurement.z(2));

  return state;
}

estimation::Covariance
MultiObjectTracker::initialize_covariance() const {
  estimation::Covariance covariance =
      estimation::Covariance::Zero();

  covariance.diagonal().head<3>().setConstant(
      config_.initial_position_variance_m2);

  covariance.diagonal().tail<3>().setConstant(
      config_.initial_velocity_variance_m2ps2);

  return covariance;
}

}  // namespace aurora::pipeline

