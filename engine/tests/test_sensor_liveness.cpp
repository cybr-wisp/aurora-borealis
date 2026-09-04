#include "ingestion/sensor_liveness.h"

#include <gtest/gtest.h>

TEST(SensorLiveness, DetectsTimeoutPerSensor) {
  aurora::ingestion::SensorLivenessMonitor monitor(1.0);

  monitor.observe(7, 10.0);

  EXPECT_FALSE(monitor.is_timed_out(7, 10.9));
  EXPECT_TRUE(monitor.is_timed_out(7, 11.1));
}

TEST(SensorLiveness, TracksSensorsIndependently) {
  aurora::ingestion::SensorLivenessMonitor monitor(1.0);

  monitor.observe(10, 5.0);
  monitor.observe(20, 5.8);

  const auto timed_out =
      monitor.timed_out_sensors(6.2);

  ASSERT_EQ(timed_out.size(), 1u);
  EXPECT_EQ(timed_out[0], 10u);

  EXPECT_TRUE(monitor.is_timed_out(10, 6.2));
  EXPECT_FALSE(monitor.is_timed_out(20, 6.2));
}

TEST(SensorLiveness, NewObservationClearsTimeout) {
  aurora::ingestion::SensorLivenessMonitor monitor(1.0);

  monitor.observe(3, 1.0);

  EXPECT_TRUE(monitor.is_timed_out(3, 2.1));

  monitor.observe(3, 2.1);

  EXPECT_FALSE(monitor.is_timed_out(3, 2.2));
}