#include "estimation/adaptive_noise.h"

#include <gtest/gtest.h>

#include <cmath>

namespace {

aurora::estimation::MeasurementCovariance configured_r() {
  aurora::estimation::MeasurementCovariance r = aurora::estimation::MeasurementCovariance::Zero();
  r.diagonal() << 16.0, 4e-6, 9e-6;
  return r;
}

aurora::estimation::MeasurementCovariance predicted_covariance() {
  aurora::estimation::MeasurementCovariance p = aurora::estimation::MeasurementCovariance::Zero();
  p.diagonal() << 9.0, 1e-6, 1e-6;
  return p;
}

aurora::estimation::MeasurementVector innovation_for_scale(
    double r_scale) {
  const auto r = configured_r();
  const auto predicted = predicted_covariance();

  aurora::estimation::MeasurementVector innovation;
  for (Eigen::Index i = 0; i < 3; ++i) {
    innovation(i) = std::sqrt(
        predicted(i, i) + r_scale * r(i, i));
  }
  return innovation;
}

}  // namespace

TEST(AdaptiveNoise, NominalInnovationsRecoverConfiguredVariance) {
  aurora::estimation::AdaptiveNoiseConfig config;
  config.window_size = 20;
  config.min_samples = 5;
  config.alpha = 0.5;

  aurora::estimation::AdaptiveMeasurementNoise estimator(config);
  const auto r = configured_r();
  const auto predicted = predicted_covariance();

  for (int i = 0; i < 20; ++i) {
    estimator.observe(
        innovation_for_scale(1.0),
        predicted,
        r);
  }

  const auto status = estimator.status(r);
  EXPECT_TRUE(status.ready);
  EXPECT_FALSE(status.degraded);
  EXPECT_NEAR(status.max_variance_ratio, 1.0, 1e-6);
}

TEST(AdaptiveNoise, DetectsSustainedSensorDegradation) {
  aurora::estimation::AdaptiveNoiseConfig config;
  config.window_size = 20;
  config.min_samples = 5;
  config.alpha = 0.25;
  config.degradation_ratio = 2.0;

  aurora::estimation::AdaptiveMeasurementNoise estimator(config);
  const auto r = configured_r();
  const auto predicted = predicted_covariance();

  for (int i = 0; i < 40; ++i) {
    estimator.observe(
        innovation_for_scale(9.0),
        predicted,
        r);
  }

  const auto status = estimator.status(r);
  EXPECT_TRUE(status.ready);
  EXPECT_TRUE(status.degraded);
  EXPECT_GT(status.max_variance_ratio, 7.0);
  EXPECT_LT(status.max_variance_ratio, 9.1);
}

TEST(AdaptiveNoise, ClampsPathologicalVarianceGrowth) {
  aurora::estimation::AdaptiveNoiseConfig config;
  config.window_size = 10;
  config.min_samples = 3;
  config.alpha = 1.0;
  config.max_scale = 4.0;

  aurora::estimation::AdaptiveMeasurementNoise estimator(config);
  const auto r = configured_r();
  const auto predicted = predicted_covariance();

  for (int i = 0; i < 10; ++i) {
    estimator.observe(
        innovation_for_scale(1000.0),
        predicted,
        r);
  }

  const auto effective = estimator.effective_r(r);
  for (Eigen::Index i = 0; i < 3; ++i) {
    EXPECT_NEAR(
        effective(i, i),
        4.0 * r(i, i),
        r(i, i) * 1e-6);
  }
}

TEST(AdaptiveNoise, MaintainsIndependentStatePerSensor) {
  aurora::estimation::AdaptiveNoiseConfig config;
  config.window_size = 10;
  config.min_samples = 3;
  config.alpha = 1.0;

  aurora::estimation::PerSensorAdaptiveNoise bank(config);
  const auto r = configured_r();
  const auto predicted = predicted_covariance();

  for (int i = 0; i < 10; ++i) {
    bank.observe(11, innovation_for_scale(1.0), predicted, r);
    bank.observe(22, innovation_for_scale(9.0), predicted, r);
  }

  const auto nominal = bank.status(11, r);
  const auto degraded = bank.status(22, r);

  EXPECT_FALSE(nominal.degraded);
  EXPECT_TRUE(degraded.degraded);
  EXPECT_LT(nominal.max_variance_ratio, 1.1);
  EXPECT_GT(degraded.max_variance_ratio, 8.0);
}

