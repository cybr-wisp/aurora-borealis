#pragma once
#include "estimation/process_model.h"
#include "estimation/measurement_model.h"
#include "estimation/update_stats.h"

namespace aurora::estimation {
class Ukf {
 public:
  Ukf(StateVector initial_state, Covariance initial_covariance, ConstantVelocityModel process_model,
      double alpha = 0.35, double beta = 2.0, double kappa = 0.0);
  void predict(double dt, double q_scale = 1.0);
  UpdateStats update(const SphericalMeasurement& measurement, const RadarMeasurementModel& model);
  const StateVector& state() const { return x_; }
  const Covariance& covariance() const { return p_; }
 private:
  static constexpr int kN = 6;
  using SigmaMatrix = Eigen::Matrix<double, kN, 2*kN+1>;
  SigmaMatrix sigma_points() const;
  StateVector x_;
  Covariance p_;
  ConstantVelocityModel process_;
  double lambda_;
  Eigen::Matrix<double, 2*kN+1, 1> wm_;
  Eigen::Matrix<double, 2*kN+1, 1> wc_;
};
}
