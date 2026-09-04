#pragma once
#include "estimation/state.h"

namespace aurora::estimation {
class ConstantVelocityModel {
 public:
  explicit ConstantVelocityModel(double accel_spectral_density = 4.0) : q_(accel_spectral_density) {}
  Eigen::Matrix<double,6,6> transition(double dt) const;
  Covariance process_noise(double dt, double scale = 1.0) const;
 private:
  double q_;
};
}
