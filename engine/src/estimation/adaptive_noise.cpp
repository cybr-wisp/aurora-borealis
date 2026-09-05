#include "estimation/adaptive_noise.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aurora::estimation {
namespace {

double clamp_value(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

}  // namespace

AdaptiveMeasurementNoise::AdaptiveMeasurementNoise(
    AdaptiveNoiseConfig config)
    : config_(std::move(config)) {
  if (config_.window_size == 0 ||
      config_.min_samples == 0 ||
      config_.min_samples > config_.window_size ||
      !(config_.alpha > 0.0 && config_.alpha <= 1.0) ||
      !(config_.min_scale > 0.0) ||
      !(config_.max_scale >= config_.min_scale) ||
      !(config_.degradation_ratio > 0.0)) {
    throw std::invalid_argument(
        "invalid adaptive measurement-noise configuration");
  }
}

void AdaptiveMeasurementNoise::validate_configured_r(
    const MeasurementCovariance& configured_r) const {
  if (!configured_r.allFinite()) {
    throw std::invalid_argument("configured R must be finite");
  }
  for (Eigen::Index i = 0; i < 3; ++i) {
    if (!(configured_r(i, i) > 0.0)) {
      throw std::invalid_argument(
          "configured R diagonal must be positive");
    }
  }
}

void AdaptiveMeasurementNoise::observe(
    const MeasurementVector& innovation,
    const MeasurementCovariance& predicted_measurement_covariance,
    const MeasurementCovariance& configured_r) {
  validate_configured_r(configured_r);

  if (!innovation.allFinite() ||
      !predicted_measurement_covariance.allFinite()) {
    throw std::invalid_argument(
        "adaptive R observation must be finite");
  }

  if (!initialized_) {
    current_r_ = configured_r;
    initialized_ = true;
  }

  Sample sample;
  for (Eigen::Index i = 0; i < 3; ++i) {
    sample.innovation_sq[static_cast<std::size_t>(i)] =
        innovation(i) * innovation(i);
    sample.predicted_variance[static_cast<std::size_t>(i)] =
        std::max(0.0, predicted_measurement_covariance(i, i));
  }

  window_.push_back(sample);
  while (window_.size() > config_.window_size) {
    window_.pop_front();
  }

  if (window_.size() < config_.min_samples) {
    return;
  }

  MeasurementCovariance candidate = configured_r;

  for (Eigen::Index i = 0; i < 3; ++i) {
    double innovation_sq_sum = 0.0;
    double predicted_variance_sum = 0.0;

    for (const auto& item : window_) {
      innovation_sq_sum +=
          item.innovation_sq[static_cast<std::size_t>(i)];
      predicted_variance_sum +=
          item.predicted_variance[static_cast<std::size_t>(i)];
    }

    const double count = static_cast<double>(window_.size());
    const double raw_variance =
        innovation_sq_sum / count -
        predicted_variance_sum / count;

    const double baseline = configured_r(i, i);
    candidate(i, i) = clamp_value(
        raw_variance,
        baseline * config_.min_scale,
        baseline * config_.max_scale);
  }

  current_r_ =
      (1.0 - config_.alpha) * current_r_ +
      config_.alpha * candidate;
  current_r_ = 0.5 * (current_r_ + current_r_.transpose());
}

MeasurementCovariance AdaptiveMeasurementNoise::effective_r(
    const MeasurementCovariance& configured_r) const {
  validate_configured_r(configured_r);
  if (!initialized_ || window_.size() < config_.min_samples) {
    return configured_r;
  }
  return current_r_;
}

AdaptiveNoiseStatus AdaptiveMeasurementNoise::status(
    const MeasurementCovariance& configured_r) const {
  validate_configured_r(configured_r);

  AdaptiveNoiseStatus result;
  result.sample_count = window_.size();
  result.ready =
      initialized_ && window_.size() >= config_.min_samples;
  result.estimated_r = result.ready ? current_r_ : configured_r;

  for (Eigen::Index i = 0; i < 3; ++i) {
    result.max_variance_ratio =
        std::max(
            result.max_variance_ratio,
            result.estimated_r(i, i) / configured_r(i, i));
  }

  result.degraded =
      result.ready &&
      result.max_variance_ratio >= config_.degradation_ratio;
  return result;
}

void AdaptiveMeasurementNoise::reset() {
  window_.clear();
  current_r_.setIdentity();
  initialized_ = false;
}

PerSensorAdaptiveNoise::PerSensorAdaptiveNoise(
    AdaptiveNoiseConfig config)
    : config_(std::move(config)) {}

MeasurementCovariance PerSensorAdaptiveNoise::effective_r(
    std::uint32_t sensor_id,
    const MeasurementCovariance& configured_r) const {
  const auto it = estimators_.find(sensor_id);
  return it == estimators_.end()
             ? configured_r
             : it->second.effective_r(configured_r);
}

void PerSensorAdaptiveNoise::observe(
    std::uint32_t sensor_id,
    const MeasurementVector& innovation,
    const MeasurementCovariance& predicted_measurement_covariance,
    const MeasurementCovariance& configured_r) {
  const auto [it, inserted] =
      estimators_.try_emplace(sensor_id, config_);
  (void)inserted;
  it->second.observe(
      innovation,
      predicted_measurement_covariance,
      configured_r);
}

AdaptiveNoiseStatus PerSensorAdaptiveNoise::status(
    std::uint32_t sensor_id,
    const MeasurementCovariance& configured_r) const {
  const auto it = estimators_.find(sensor_id);
  if (it == estimators_.end()) {
    AdaptiveNoiseStatus result;
    result.estimated_r = configured_r;
    return result;
  }
  return it->second.status(configured_r);
}

void PerSensorAdaptiveNoise::reset_sensor(std::uint32_t sensor_id) {
  estimators_.erase(sensor_id);
}

void PerSensorAdaptiveNoise::reset_all() {
  estimators_.clear();
}

}  // namespace aurora::estimation
