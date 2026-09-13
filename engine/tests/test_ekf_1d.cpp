#include "estimation/ekf.h"

#include <gtest/gtest.h>

#include <cmath>

TEST(Ekf, ReducesPositionUncertaintyAfterMeasurement) {
  using namespace aurora::estimation;

  StateVector initial_state;
  initial_state << 1000, 300, 200, 50, 0, 0;

  const Covariance initial_covariance =
      Covariance::Identity() * 400.0;

  Ekf filter(
      initial_state,
      initial_covariance,
      ConstantVelocityModel(2.0));

  filter.predict(0.1);

  RadarMeasurementModel radar(Eigen::Vector3d::Zero());

  MeasurementVector measurement = radar.predict(filter.state());
  measurement(0) += 5.0;

  MeasurementCovariance measurement_noise =
      MeasurementCovariance::Zero();
  measurement_noise.diagonal() << 100.0, 1e-5, 1e-5;

  const double uncertainty_before =
      filter.covariance().topLeftCorner<3, 3>().trace();

  const UpdateStats stats =
      filter.update(
          {measurement, measurement_noise},
          radar);

  const double uncertainty_after =
      filter.covariance().topLeftCorner<3, 3>().trace();

  EXPECT_LT(uncertainty_after, uncertainty_before);
  EXPECT_TRUE(filter.state().allFinite());
  EXPECT_TRUE(filter.covariance().allFinite());

  const double symmetry_error =
      (filter.covariance() -
       filter.covariance().transpose())
          .norm();

  EXPECT_LT(symmetry_error, 1e-10);
  EXPECT_TRUE(std::isfinite(stats.nis));
  EXPECT_GE(stats.nis, 0.0);
}