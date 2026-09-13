#include "estimation/ukf.h"

#include "geometry/conversions.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace aurora::estimation {

Ukf::Ukf(
    StateVector initial_state,
    Covariance initial_covariance,
    ConstantVelocityModel process_model,
    double alpha,
    double beta,
    double kappa)
    : x_(std::move(initial_state)),
      p_(std::move(initial_covariance)),
      process_(std::move(process_model)) {
  lambda_ = alpha * alpha * (kN + kappa) - kN;

  const double sigma_scale = kN + lambda_;
  wm_.setConstant(1.0 / (2.0 * sigma_scale));
  wc_ = wm_;

  wm_(0) = lambda_ / sigma_scale;
  wc_(0) = wm_(0) + (1.0 - alpha * alpha + beta);
}

Ukf::SigmaMatrix Ukf::sigma_points() const {
  Eigen::LLT<Covariance> factor((kN + lambda_) * p_);
  if (factor.info() != Eigen::Success) {
    throw std::runtime_error("UKF covariance not positive definite");
  }

  SigmaMatrix sigmas;
  sigmas.col(0) = x_;

  const Covariance root = factor.matrixL();

  for (int i = 0; i < kN; ++i) {
    sigmas.col(i + 1) = x_ + root.col(i);
    sigmas.col(i + 1 + kN) = x_ - root.col(i);
  }

  return sigmas;
}

void Ukf::predict(double dt, double q_scale) {
  const auto transition = process_.transition(dt);
  auto sigmas = sigma_points();

  for (int i = 0; i < sigmas.cols(); ++i) {
    sigmas.col(i) = transition * sigmas.col(i);
  }

  x_.setZero();
  for (int i = 0; i < sigmas.cols(); ++i) {
    x_ += wm_(i) * sigmas.col(i);
  }

  p_ = process_.process_noise(dt, q_scale);

  for (int i = 0; i < sigmas.cols(); ++i) {
    const StateVector delta = sigmas.col(i) - x_;
    p_ += wc_(i) * (delta * delta.transpose());
  }

  p_ = 0.5 * (p_ + p_.transpose());
}

UpdateStats Ukf::update(
    const SphericalMeasurement& measurement,
    const RadarMeasurementModel& model) {
  const auto sigmas = sigma_points();

  Eigen::Matrix<double, 3, 2 * kN + 1> predicted_measurements;
  for (int i = 0; i < sigmas.cols(); ++i) {
    predicted_measurements.col(i) = model.predict(sigmas.col(i));
  }

  MeasurementVector measurement_mean = MeasurementVector::Zero();
  measurement_mean(0) =
      wm_.dot(predicted_measurements.row(0).transpose());

  // Angular components are averaged on the unit circle across the wrap boundary.
  for (int angle_index : {1, 2}) {
    double weighted_sine = 0.0;
    double weighted_cosine = 0.0;

    for (int i = 0; i < predicted_measurements.cols(); ++i) {
      const double angle = predicted_measurements(angle_index, i);
      weighted_sine += wm_(i) * std::sin(angle);
      weighted_cosine += wm_(i) * std::cos(angle);
    }

    measurement_mean(angle_index) =
        std::atan2(weighted_sine, weighted_cosine);
  }

  MeasurementCovariance innovation_covariance = measurement.r;
  Eigen::Matrix<double, 6, 3> cross_covariance =
      Eigen::Matrix<double, 6, 3>::Zero();

  for (int i = 0; i < predicted_measurements.cols(); ++i) {
    const StateVector state_delta = sigmas.col(i) - x_;
    const MeasurementVector measurement_delta =
        model.residual(
            predicted_measurements.col(i),
            measurement_mean);

    innovation_covariance +=
        wc_(i) *
        (measurement_delta * measurement_delta.transpose());

    cross_covariance +=
        wc_(i) *
        (state_delta * measurement_delta.transpose());
  }

  Eigen::LDLT<MeasurementCovariance> factor(
      innovation_covariance);

  if (factor.info() != Eigen::Success) {
    throw std::runtime_error(
        "UKF innovation covariance factorization failed");
  }

  const Eigen::Matrix<double, 6, 3> gain =
      cross_covariance *
      factor.solve(MeasurementCovariance::Identity());

  const MeasurementVector innovation =
      model.residual(
          measurement.z,
          measurement_mean);

  x_ += gain * innovation;
  p_ -=
      gain *
      innovation_covariance *
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