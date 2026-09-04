#pragma once
#include "estimation/state.h"
#include <Eigen/Dense>
#include <utility>

namespace aurora::estimation {
struct SphericalMeasurement {
  MeasurementVector z;
  MeasurementCovariance r;
};

class RadarMeasurementModel {
 public:
  explicit RadarMeasurementModel(Eigen::Vector3d sensor_position_enu_m) : sensor_(std::move(sensor_position_enu_m)) {}
  MeasurementVector predict(const StateVector& x) const;
  MeasurementJacobian jacobian(const StateVector& x) const;
  MeasurementVector residual(const MeasurementVector& observed, const MeasurementVector& predicted) const;
 private:
  Eigen::Vector3d sensor_;
};
}
