#include "estimation/ukf.h"
#include "geometry/conversions.h"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aurora::estimation {
Ukf::Ukf(StateVector x, Covariance p, ConstantVelocityModel process, double alpha, double beta, double kappa)
    : x_(std::move(x)), p_(std::move(p)), process_(std::move(process)) {
  lambda_ = alpha*alpha*(kN + kappa) - kN;
  wm_.setConstant(1.0 / (2.0 * (kN + lambda_)));
  wc_ = wm_;
  wm_(0) = lambda_ / (kN + lambda_);
  wc_(0) = wm_(0) + (1.0 - alpha*alpha + beta);
}
Ukf::SigmaMatrix Ukf::sigma_points() const {
  Eigen::LLT<Covariance> llt((kN + lambda_) * p_);
  if (llt.info() != Eigen::Success) throw std::runtime_error("UKF covariance not positive definite");
  SigmaMatrix sigmas;
  sigmas.col(0) = x_;
  const Covariance l = llt.matrixL();
  for (int i = 0; i < kN; ++i) {
    sigmas.col(i+1) = x_ + l.col(i);
    sigmas.col(i+1+kN) = x_ - l.col(i);
  }
  return sigmas;
}
void Ukf::predict(double dt, double q_scale) {
  const auto f = process_.transition(dt);
  auto sigmas = sigma_points();
  for (int i=0;i<sigmas.cols();++i) sigmas.col(i) = f * sigmas.col(i);
  x_.setZero();
  for (int i=0;i<sigmas.cols();++i) x_ += wm_(i)*sigmas.col(i);
  p_ = process_.process_noise(dt, q_scale);
  for (int i=0;i<sigmas.cols();++i) { const StateVector d=sigmas.col(i)-x_; p_ += wc_(i)*(d*d.transpose()); }
  p_ = 0.5*(p_+p_.transpose());
}
UpdateStats Ukf::update(const SphericalMeasurement& m, const RadarMeasurementModel& model) {
  const auto sigmas = sigma_points();
  Eigen::Matrix<double,3,2*kN+1> zs;
  for (int i=0;i<sigmas.cols();++i) zs.col(i)=model.predict(sigmas.col(i));
  MeasurementVector zmean = MeasurementVector::Zero();
  zmean(0) = wm_.dot(zs.row(0).transpose());
  for (int angle_idx : {1,2}) {
    double s=0.0,c=0.0;
    for (int i=0;i<zs.cols();++i) { s += wm_(i)*std::sin(zs(angle_idx,i)); c += wm_(i)*std::cos(zs(angle_idx,i)); }
    zmean(angle_idx)=std::atan2(s,c);
  }
  MeasurementCovariance s = m.r;
  Eigen::Matrix<double,6,3> pxz = Eigen::Matrix<double,6,3>::Zero();
  for (int i=0;i<zs.cols();++i) {
    const StateVector dx=sigmas.col(i)-x_;
    const MeasurementVector dz=model.residual(zs.col(i), zmean);
    s += wc_(i)*(dz*dz.transpose());
    pxz += wc_(i)*(dx*dz.transpose());
  }
  Eigen::LDLT<MeasurementCovariance> ldlt(s);
  if (ldlt.info()!=Eigen::Success) throw std::runtime_error("UKF innovation covariance factorization failed");
  const auto k = pxz * ldlt.solve(MeasurementCovariance::Identity());
  const auto innovation=model.residual(m.z,zmean);
  x_ += k*innovation;
  p_ -= k*s*k.transpose();
  p_ = 0.5*(p_+p_.transpose());
  const double nis=innovation.dot(ldlt.solve(innovation));
  return {innovation,s,nis};
}
}
