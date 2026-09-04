#include "estimation/ekf.h"
#include <gtest/gtest.h>
#include <cmath>
TEST(Ekf, ReducesPositionUncertaintyAfterMeasurement){
  using namespace aurora::estimation;
  StateVector x; x<<1000,300,200,50,0,0; Covariance p=Covariance::Identity()*400.0;
  Ekf f(x,p,ConstantVelocityModel(2.0)); f.predict(0.1);
  RadarMeasurementModel model(Eigen::Vector3d::Zero()); auto z=model.predict(f.state());
  z(0)+=5.0;
  MeasurementCovariance r=MeasurementCovariance::Zero(); r.diagonal()<<100.0,1e-5,1e-5;
  const double before=f.covariance().topLeftCorner<3,3>().trace();
  f.update({z,r},model);
  EXPECT_LT(f.covariance().topLeftCorner<3,3>().trace(),before);
}
