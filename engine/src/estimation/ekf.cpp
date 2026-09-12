#include "estimation/ekf.h"

#include <stdexcept>
#include <utility>

namespace aurora::estimation {

Ekf::Ekf(
    StateVector initial_state,
    Covariance initial_covariance,
    ConstantVelocityModel process_model)
    : x_(std::move(initial_state)),
      p_(std::move(initial_covariance)),
      process_(std::move(process_model)) {}

void Ekf::predict(double dt, double q_scale) {
  const auto transition = process_.transition(dt);
  const auto process_noise = process_.process_noise(dt, q_scale);

  x_ = transition * x_;
  p_ =
      transition * p_ * transition.transpose() +
      process_noise;

  // Numerical drift can make P very slightly asymmetric over long runs.
  p_ = 0.5 * (p_ + p_.transpose());
}

UpdateStats Ekf::update(
    const SphericalMeasurement& measurement,
    const RadarMeasurementModel& model) {
  const auto jacobian = model.jacobian(x_);
  const auto predicted_measurement = model.predict(x_);
  const auto innovation =
      model.residual(
          measurement.z,
          predicted_measurement);

  const MeasurementCovariance innovation_covariance =
      jacobian * p_ * jacobian.transpose() +
      measurement.r;

  Eigen::LDLT<MeasurementCovariance> factor(
      innovation_covariance);

  if (factor.info() != Eigen::Success) {
    throw std::runtime_error(
        "innovation covariance factorization failed");
  }

  const Eigen::Matrix<double, 6, 3> gain =
      p_ *
      jacobian.transpose() *
      factor.solve(
          MeasurementCovariance::Identity());

  x_ += gain * innovation;

  // Joseph form costs a little more than P=(I-KH)P, but behaves better
  // when covariance consistency is one of the quantities being measured.
  const Covariance identity = Covariance::Identity();
  const Covariance correction =
      identity - gain * jacobian;

  p_ =
      correction *
          p_ *
          correction.transpose() +
      gain *
          measurement.r *
          gain.transpose();

  p_ = 0.5 * (p_ + p_.transpose());

  const double nis =
      innovation.dot(
          factor.solve(innovation));

  return {
      innovation,
      innovation_covariance,
      nis,
  };
}

}  // namespace aurora::estimation
