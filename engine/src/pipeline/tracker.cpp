#include "pipeline/tracker.h"

#include "association/gating.h"
#include "association/hungarian.h"
#include "estimation/ekf.h"
#include "geometry/conversions.h"

#include <Eigen/Dense>

#include <map>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace aurora::pipeline {
namespace {

bool same_position(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b) {
  return (a - b).norm() <= 1e-9;
}

}  // namespace

MultiObjectTracker::MultiObjectTracker(TrackerConfig config)
    : config_(config),
      tracks_(association::TrackManagerConfig{
          config.confirmation_hits,
          config.coast_after_misses,
          config.tentative_delete_misses,
          config.delete_after_misses}),
      process_(config.accel_spectral_density),
      adaptive_noise_(config.adaptive_noise) {
  if (!(config_.gate_threshold > 0.0) ||
      !(config_.initial_position_variance_m2 > 0.0) ||
      !(config_.initial_velocity_variance_m2ps2 > 0.0)) {
    throw std::invalid_argument(
        "invalid multi-object tracker configuration");
  }
}

FrameResult MultiObjectTracker::process_frame(
    const std::vector<estimation::SphericalMeasurement>& measurements,
    const Eigen::Vector3d& sensor_position_enu_m,
    double dt_sec) {
  std::vector<SensorMeasurement> sourced;
  sourced.reserve(measurements.size());

  for (const auto& measurement : measurements) {
    sourced.push_back(
        SensorMeasurement{0, sensor_position_enu_m, measurement});
  }

  return process_sensor_frame(sourced, dt_sec);
}

FrameResult MultiObjectTracker::process_sensor_frame(
    const std::vector<SensorMeasurement>& measurements,
    double dt_sec) {
  if (!(dt_sec > 0.0)) {
    throw std::invalid_argument("dt_sec must be positive");
  }

  FrameResult result;

  const auto frame_start_track_ids = tracks_.active_track_ids();
  const std::unordered_set<std::uint64_t> frame_start_track_set(
      frame_start_track_ids.begin(),
      frame_start_track_ids.end());

  const auto f = process_.transition(dt_sec);
  const auto q = process_.process_noise(dt_sec);

  for (const auto track_id : frame_start_track_ids) {
    auto* track = tracks_.find(track_id);
    if (track == nullptr) {
      continue;
    }

    const auto predicted_state = f * track->state;
    estimation::Covariance predicted_covariance =
        f * track->covariance * f.transpose() + q;
    predicted_covariance =
        0.5 * (predicted_covariance +
               predicted_covariance.transpose());

    tracks_.update_estimate(
        track_id,
        predicted_state,
        predicted_covariance);
  }

  std::map<std::uint32_t, std::vector<std::size_t>> sensor_groups;
  std::unordered_map<std::uint32_t, Eigen::Vector3d> sensor_positions;

  for (std::size_t index = 0; index < measurements.size(); ++index) {
    const auto& sourced = measurements[index];

    if (!sourced.sensor_position_enu_m.allFinite()) {
      throw std::invalid_argument("sensor position must be finite");
    }

    const auto position_it = sensor_positions.find(sourced.sensor_id);
    if (position_it == sensor_positions.end()) {
      sensor_positions.emplace(
          sourced.sensor_id,
          sourced.sensor_position_enu_m);
    } else if (!same_position(
                   position_it->second,
                   sourced.sensor_position_enu_m)) {
      throw std::invalid_argument(
          "one sensor_id cannot have multiple positions within a frame");
    }

    sensor_groups[sourced.sensor_id].push_back(index);
  }

  std::unordered_set<std::uint64_t> associated_existing_tracks;
  std::unordered_set<std::size_t> assigned_measurements;

  for (const auto& [sensor_id, indices] : sensor_groups) {
    const auto current_track_ids = tracks_.active_track_ids();
    const auto sensor_position = sensor_positions.at(sensor_id);
    const estimation::RadarMeasurementModel model(sensor_position);

    Eigen::MatrixXd costs(
        static_cast<Eigen::Index>(current_track_ids.size()),
        static_cast<Eigen::Index>(indices.size()));
    costs.setConstant(config_.gate_threshold + 1.0);

    for (std::size_t row = 0; row < current_track_ids.size(); ++row) {
      const auto* track = tracks_.find(current_track_ids[row]);
      if (track == nullptr) {
        continue;
      }

      for (std::size_t local_col = 0;
           local_col < indices.size();
           ++local_col) {
        const std::size_t global_col = indices[local_col];
        auto effective = measurements[global_col].measurement;

        if (config_.enable_adaptive_noise) {
          effective.r = adaptive_noise_.effective_r(
              sensor_id,
              measurements[global_col].measurement.r);
        }

        const auto gate = association::gate_measurement(
            track->state,
            track->covariance,
            effective,
            model,
            config_.gate_threshold);

        if (gate.accepted) {
          costs(
              static_cast<Eigen::Index>(row),
              static_cast<Eigen::Index>(local_col)) =
              gate.mahalanobis_sq;
          ++result.gated_pair_count;
        }
      }
    }

    const auto assignment = association::hungarian_assign(
        costs,
        config_.gate_threshold);

    for (std::size_t row = 0; row < current_track_ids.size(); ++row) {
      const int assigned_local_col =
          assignment.empty() ? -1 : assignment[row];

      if (assigned_local_col < 0) {
        continue;
      }

      const auto local_col =
          static_cast<std::size_t>(assigned_local_col);
      const auto global_col = indices[local_col];

      auto* track = tracks_.find(current_track_ids[row]);
      if (track == nullptr) {
        continue;
      }

      auto effective = measurements[global_col].measurement;
      if (config_.enable_adaptive_noise) {
        effective.r = adaptive_noise_.effective_r(
            sensor_id,
            measurements[global_col].measurement.r);
      }

      const auto h = model.jacobian(track->state);
      const auto zhat = model.predict(track->state);
      const auto innovation = model.residual(
          measurements[global_col].measurement.z,
          zhat);
      const estimation::MeasurementCovariance predicted_meas_cov =
          h * track->covariance * h.transpose();

      estimation::Ekf ekf(
          track->state,
          track->covariance,
          process_);
      ekf.update(effective, model);

      tracks_.update_estimate(
          current_track_ids[row],
          ekf.state(),
          ekf.covariance());

      if (frame_start_track_set.contains(current_track_ids[row])) {
        associated_existing_tracks.insert(current_track_ids[row]);
      }

      assigned_measurements.insert(global_col);

      if (config_.enable_adaptive_noise) {
        adaptive_noise_.observe(
            sensor_id,
            innovation,
            predicted_meas_cov,
            measurements[global_col].measurement.r);
      }

      result.assignments.push_back({
          current_track_ids[row],
          global_col,
          sensor_id,
          costs(
              static_cast<Eigen::Index>(row),
              static_cast<Eigen::Index>(local_col))});
    }

    for (const auto global_col : indices) {
      if (assigned_measurements.contains(global_col)) {
        continue;
      }

      const auto track_id = tracks_.create(
          initialize_state(
              measurements[global_col].measurement,
              sensor_position),
          initialize_covariance());

      result.created_tracks.push_back(track_id);
      result.unassigned_measurements.push_back(global_col);
    }
  }

  for (const auto track_id : frame_start_track_ids) {
    auto* track = tracks_.find(track_id);
    if (track == nullptr) {
      continue;
    }

    if (associated_existing_tracks.contains(track_id)) {
      tracks_.mark_associated(
          track_id,
          track->state,
          track->covariance);
    } else {
      tracks_.mark_missed(track_id);
    }
  }

  for (const auto& [sensor_id, indices] : sensor_groups) {
    if (indices.empty()) {
      continue;
    }

    const auto& configured_r =
        measurements[indices.front()].measurement.r;
    const auto status =
        adaptive_noise_.status(sensor_id, configured_r);

    result.sensor_noise.push_back({
        sensor_id,
        status.estimated_r,
        status.sample_count,
        status.ready,
        status.degraded,
        status.max_variance_ratio});
  }

  tracks_.prune_deleted();
  return result;
}

estimation::StateVector MultiObjectTracker::initialize_state(
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

estimation::Covariance MultiObjectTracker::initialize_covariance() const {
  estimation::Covariance covariance =
      estimation::Covariance::Zero();

  covariance.diagonal().head<3>().setConstant(
      config_.initial_position_variance_m2);
  covariance.diagonal().tail<3>().setConstant(
      config_.initial_velocity_variance_m2ps2);

  return covariance;
}

}  // namespace aurora::pipeline
