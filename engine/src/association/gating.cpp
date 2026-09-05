#include "association/gating.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aurora::association {

double mahalanobis_squared(
    const estimation::MeasurementVector& innovation,
    const estimation::MeasurementCovariance& innovation_covariance) {
  Eigen::LDLT<estimation::MeasurementCovariance> ldlt(
      innovation_covariance);

  if (ldlt.info() != Eigen::Success ||
      (ldlt.vectorD().array() <= 0.0).any()) {
    return std::numeric_limits<double>::infinity();
  }

  const auto solved = ldlt.solve(innovation);
  if (ldlt.info() != Eigen::Success || !solved.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }

  const double d2 = innovation.dot(solved);
  if (!std::isfinite(d2) || d2 < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return d2;
}

GateResult gate_measurement(
    const estimation::StateVector& predicted_state,
    const estimation::Covariance& predicted_covariance,
    const estimation::SphericalMeasurement& measurement,
    const estimation::RadarMeasurementModel& model,
    double threshold) {
  if (!(threshold > 0.0) || !std::isfinite(threshold)) {
    throw std::invalid_argument(
        "gating threshold must be finite and positive");
  }

  try {
    const auto predicted_measurement =
        model.predict(predicted_state);
    const auto h = model.jacobian(predicted_state);
    const auto innovation =
        model.residual(measurement.z, predicted_measurement);
    const estimation::MeasurementCovariance s =
        h * predicted_covariance * h.transpose() +
        measurement.r;

    const double d2 = mahalanobis_squared(innovation, s);
    return {d2, d2 <= threshold};
  } catch (const std::exception&) {
    return {
        std::numeric_limits<double>::infinity(),
        false};
  }
}

}  // namespace aurora::association
