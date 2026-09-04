#include "estimation/ekf.h"
#include <stdexcept>
#include <utility>

namespace aurora::estimation {
Ekf::Ekf(StateVector x, Covariance p, ConstantVelocityModel process) : x_(std::move(x)), p_(std::move(p)), process_(std::move(process)) {}
void Ekf::predict(double dt, double q_scale) {
  const auto f = process_.transition(dt);
  x_ = f * x_;
  p_ = f * p_ * f.transpose() + process_.process_noise(dt, q_scale);
  p_ = 0.5 * (p_ + p_.transpose());
}
UpdateStats Ekf::update(const SphericalMeasurement& m, const RadarMeasurementModel& model) {
  const auto h = model.jacobian(x_);
  const auto zhat = model.predict(x_);
  const auto innovation = model.residual(m.z, zhat);
  const MeasurementCovariance s = h * p_ * h.transpose() + m.r;
  Eigen::LDLT<MeasurementCovariance> ldlt(s);
  if (ldlt.info() != Eigen::Success) throw std::runtime_error("innovation covariance factorization failed");
  const Eigen::Matrix<double,6,3> k = p_ * h.transpose() * ldlt.solve(MeasurementCovariance::Identity());
  x_ += k * innovation;
  const Covariance i = Covariance::Identity();
  const Covariance ikh = i - k * h;
  p_ = ikh * p_ * ikh.transpose() + k * m.r * k.transpose();
  p_ = 0.5 * (p_ + p_.transpose());
  const double nis = innovation.dot(ldlt.solve(innovation));
  return {innovation, s, nis};
}
}
