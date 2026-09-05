#pragma once

#include "estimation/measurement_model.h"
#include "estimation/state.h"

namespace aurora::association {

inline constexpr double kChiSquareGate95Measurement3D =
    7.814727903251179;

struct GateResult {
  double mahalanobis_sq{0.0};
  bool accepted{false};
};

double mahalanobis_squared(
    const estimation::MeasurementVector& innovation,
    const estimation::MeasurementCovariance& innovation_covariance);

GateResult gate_measurement(
    const estimation::StateVector& predicted_state,
    const estimation::Covariance& predicted_covariance,
    const estimation::SphericalMeasurement& measurement,
    const estimation::RadarMeasurementModel& model,
    double threshold = kChiSquareGate95Measurement3D);

}  // namespace aurora::association
