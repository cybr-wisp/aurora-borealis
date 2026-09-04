#include "estimation/measurement_model.h"
#include "geometry/conversions.h"
#include <cmath>
#include <stdexcept>

namespace aurora::estimation {
MeasurementVector RadarMeasurementModel::predict(const StateVector& x) const {
  return geometry::cartesian_to_spherical(x.head<3>() - sensor_);
}
MeasurementJacobian RadarMeasurementModel::jacobian(const StateVector& x) const {
  const Eigen::Vector3d d = x.head<3>() - sensor_;
  const double dx=d.x(), dy=d.y(), dz=d.z();
  const double rho2=dx*dx+dy*dy, rho=std::sqrt(rho2), r2=rho2+dz*dz, r=std::sqrt(r2);
  if (rho2 < 1e-12 || r2 < 1e-12) throw std::invalid_argument("measurement Jacobian singular geometry");
  MeasurementJacobian h = MeasurementJacobian::Zero();
  h(0,0)=dx/r; h(0,1)=dy/r; h(0,2)=dz/r;
  h(1,0)=-dy/rho2; h(1,1)=dx/rho2;
  h(2,0)=-dx*dz/(r2*rho); h(2,1)=-dy*dz/(r2*rho); h(2,2)=rho/r2;
  return h;
}
MeasurementVector RadarMeasurementModel::residual(const MeasurementVector& observed, const MeasurementVector& predicted) const {
  MeasurementVector y = observed - predicted;
  y(1) = geometry::wrap_angle(y(1));
  y(2) = geometry::wrap_angle(y(2));
  return y;
}
}
