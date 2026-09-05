#include "pipeline/tracker.h"
#include "geometry/conversions.h"

#include <gtest/gtest.h>

namespace {

aurora::estimation::SphericalMeasurement make_measurement(
    const Eigen::Vector3d& position) {
  aurora::estimation::SphericalMeasurement measurement{};
  measurement.z =
      aurora::geometry::cartesian_to_spherical(position);
  measurement.r.setZero();
  measurement.r.diagonal() << 4.0, 1e-5, 1e-5;
  return measurement;
}

}  // namespace

TEST(MultiObjectTracker, ConfirmsTwoIndependentTracks) {
  aurora::pipeline::TrackerConfig config;
  config.confirmation_hits = 3;
  config.delete_after_misses = 3;

  aurora::pipeline::MultiObjectTracker tracker(config);

  const Eigen::Vector3d sensor =
      Eigen::Vector3d::Zero();

  const std::vector<
      aurora::estimation::SphericalMeasurement>
      measurements = {
          make_measurement(Eigen::Vector3d(1000.0, -60.0, 25.0)),
          make_measurement(Eigen::Vector3d(1000.0, 60.0, 25.0))};

  tracker.process_frame(measurements, sensor, 0.1);
  ASSERT_EQ(tracker.tracks().size(), 2U);

  tracker.process_frame(measurements, sensor, 0.1);
  tracker.process_frame(measurements, sensor, 0.1);

  ASSERT_EQ(tracker.tracks().size(), 2U);

  for (const auto& track : tracker.tracks()) {
    EXPECT_EQ(
        track.status,
        aurora::association::TrackStatus::kConfirmed);
  }
}

TEST(MultiObjectTracker, ConfirmedTrackCoastsThenDeletes) {
  aurora::pipeline::TrackerConfig config;
  config.confirmation_hits = 1;
  config.delete_after_misses = 2;

  aurora::pipeline::MultiObjectTracker tracker(config);

  const Eigen::Vector3d sensor =
      Eigen::Vector3d::Zero();

  tracker.process_frame(
      {make_measurement(Eigen::Vector3d(1000.0, 0.0, 25.0))},
      sensor,
      0.1);

  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_EQ(
      tracker.tracks().front().status,
      aurora::association::TrackStatus::kConfirmed);

  tracker.process_frame({}, sensor, 0.1);

  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_EQ(
      tracker.tracks().front().status,
      aurora::association::TrackStatus::kCoasting);

  tracker.process_frame({}, sensor, 0.1);
  EXPECT_TRUE(tracker.tracks().empty());
}
