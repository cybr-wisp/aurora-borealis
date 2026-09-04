#pragma once
#include "estimation/process_model.h"
#include "estimation/measurement_model.h"

namespace aurora::estimation {
struct UpdateStats { MeasurementVector innovation; MeasurementCovariance innovation_covariance; double nis; };

class Ekf {
 public:
  Ekf(StateVector initial_state, Covariance initial_covariance, ConstantVelocityModel process_model);
  void predict(double dt, double q_scale = 1.0);
  UpdateStats update(const SphericalMeasurement& measurement, const RadarMeasurementModel& model);
  const StateVector& state() const { return x_; }
  const Covariance& covariance() const { return p_; }
 private:
  StateVector x_;
  Covariance p_;
  ConstantVelocityModel process_;
};
}
