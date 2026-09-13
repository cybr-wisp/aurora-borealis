#include "estimation/ukf.h"

#include <gtest/gtest.h>

#include <cmath>

TEST(Ukf, MaintainsSymmetricCovarianceAfterUpdate) {
  using namespace aurora::estimation;

  StateVector initial_state;
  initial_state << 1000, 250, 180, 45, 5, 0;

  const Covariance initial_covariance =
      Covariance::Identity() * 300.0;

  Ukf filter(
      initial_state,
      initial_covariance,
      ConstantVelocityModel(3.0));

  filter.predict(0.1);

  RadarMeasurementModel radar(Eigen::Vector3d::Zero());

  MeasurementVector measurement = radar.predict(filter.state());
  measurement(1) += 0.001;

  MeasurementCovariance measurement_noise =
      MeasurementCovariance::Zero();
  measurement_noise.diagonal() << 100.0, 1e-5, 1e-5;

  const UpdateStats stats =
      filter.update(
          {measurement, measurement_noise},
          radar);

  const Covariance& covariance = filter.covariance();

  EXPECT_TRUE(filter.state().allFinite());
  EXPECT_TRUE(covariance.allFinite());

  const double symmetry_error =
      (covariance - covariance.transpose()).norm();

  EXPECT_LT(symmetry_error, 1e-10);
  EXPECT_TRUE(std::isfinite(stats.nis));
  EXPECT_GE(stats.nis, 0.0);
}