#include "pipeline/tracker.h"
#include "geometry/conversions.h"

#include <gtest/gtest.h>

namespace {

aurora::estimation::SphericalMeasurement make_measurement(
    const Eigen::Vector3d& target,
    const Eigen::Vector3d& sensor) {
  aurora::estimation::SphericalMeasurement measurement{};
  measurement.z =
      aurora::geometry::cartesian_to_spherical(target - sensor);
  measurement.r.setZero();
  measurement.r.diagonal() << 4.0, 1e-5, 1e-5;
  return measurement;
}

aurora::pipeline::SensorMeasurement source(
    std::uint32_t sensor_id,
    const Eigen::Vector3d& sensor,
    const Eigen::Vector3d& target) {
  return {
      sensor_id,
      sensor,
      make_measurement(target, sensor)};
}

}  // namespace

TEST(MultiSensorTracker, FusesTwoSensorsWithoutDoubleLifecycleAdvance) {
  aurora::pipeline::TrackerConfig config;
  config.confirmation_hits = 2;

  aurora::pipeline::MultiObjectTracker tracker(config);

  const Eigen::Vector3d sensor_a(0.0, 0.0, 0.0);
  const Eigen::Vector3d sensor_b(100.0, 0.0, 0.0);
  const Eigen::Vector3d target(1000.0, 80.0, 20.0);

  tracker.process_sensor_frame(
      {source(11, sensor_a, target)},
      0.1);

  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_EQ(tracker.tracks().front().age_frames, 1);

  const auto result = tracker.process_sensor_frame(
      {
          source(11, sensor_a, target),
          source(22, sensor_b, target),
      },
      0.1);

  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_EQ(result.assignments.size(), 2U);
  EXPECT_EQ(tracker.tracks().front().age_frames, 2);
  EXPECT_EQ(
      tracker.tracks().front().status,
      aurora::association::TrackStatus::kConfirmed);
}

TEST(MultiSensorTracker, SurvivingSensorPreventsCoasting) {
  aurora::pipeline::TrackerConfig config;
  config.confirmation_hits = 1;
  config.delete_after_misses = 2;

  aurora::pipeline::MultiObjectTracker tracker(config);

  const Eigen::Vector3d sensor_a(0.0, 0.0, 0.0);
  const Eigen::Vector3d sensor_b(100.0, 0.0, 0.0);
  const Eigen::Vector3d target(900.0, 20.0, 10.0);

  tracker.process_sensor_frame(
      {
          source(11, sensor_a, target),
          source(22, sensor_b, target),
      },
      0.1);

  tracker.process_sensor_frame(
      {source(11, sensor_a, target)},
      0.1);

  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_EQ(
      tracker.tracks().front().status,
      aurora::association::TrackStatus::kConfirmed);
  EXPECT_EQ(tracker.tracks().front().consecutive_misses, 0);
}
