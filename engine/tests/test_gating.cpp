#include "association/gating.h"
#include "estimation/measurement_model.h"
#include "estimation/state.h"

#include <gtest/gtest.h>

TEST(Gating, AcceptsNearMeasurementAndRejectsFarMeasurement) {
  aurora::estimation::StateVector x =
      aurora::estimation::StateVector::Zero();
  x << 1000.0, 100.0, 50.0, 0.0, 0.0, 0.0;

  aurora::estimation::Covariance p =
      aurora::estimation::Covariance::Identity() * 25.0;

  aurora::estimation::RadarMeasurementModel model(
      Eigen::Vector3d::Zero());

  aurora::estimation::SphericalMeasurement near{};
  near.z = model.predict(x);
  near.z(0) += 1.0;
  near.r.setZero();
  near.r.diagonal() << 16.0, 1e-5, 1e-5;

  const auto accepted =
      aurora::association::gate_measurement(
          x, p, near, model);

  EXPECT_TRUE(accepted.accepted);
  EXPECT_LT(
      accepted.mahalanobis_sq,
      aurora::association::
          kChiSquareGate95Measurement3D);

  auto far = near;
  far.z(0) += 500.0;

  const auto rejected =
      aurora::association::gate_measurement(
          x, p, far, model);

  EXPECT_FALSE(rejected.accepted);
}
