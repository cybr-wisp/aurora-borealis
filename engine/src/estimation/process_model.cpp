#include "estimation/process_model.h"
#include <stdexcept>

namespace aurora::estimation {
Eigen::Matrix<double,6,6> ConstantVelocityModel::transition(double dt) const {
  if (dt < 0.0) throw std::invalid_argument("negative dt");
  Covariance f = Covariance::Identity();
  for (int i = 0; i < 3; ++i) f(i, i + 3) = dt;
  return f;
}
Covariance ConstantVelocityModel::process_noise(double dt, double scale) const {
  if (dt < 0.0 || scale <= 0.0) throw std::invalid_argument("invalid process noise arguments");
  Covariance q = Covariance::Zero();
  const double qv = q_ * scale;
  const double a = qv * dt*dt*dt / 3.0;
  const double b = qv * dt*dt / 2.0;
  const double c = qv * dt;
  for (int i = 0; i < 3; ++i) {
    q(i,i)=a; q(i,i+3)=b; q(i+3,i)=b; q(i+3,i+3)=c;
  }
  return q;
}
}

