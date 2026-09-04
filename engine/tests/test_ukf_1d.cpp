#include "estimation/ukf.h"
#include <gtest/gtest.h>

TEST(Ukf, MaintainsSymmetricCovarianceAfterUpdate){
  using namespace aurora::estimation;
  StateVector x; x<<1000,250,180,45,5,0; Covariance p=Covariance::Identity()*300.0;
  Ukf f(x,p,ConstantVelocityModel(3.0)); f.predict(0.1);
  RadarMeasurementModel model(Eigen::Vector3d::Zero()); auto z=model.predict(f.state()); z(1)+=0.001;
  MeasurementCovariance r=MeasurementCovariance::Zero(); r.diagonal()<<100.0,1e-5,1e-5;
  f.update({z,r},model);
  EXPECT_LT((f.covariance()-f.covariance().transpose()).norm(),1e-10);
  EXPECT_TRUE(f.covariance().allFinite());
}
