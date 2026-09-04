#include "estimation/maneuver_detector.h"
#include <gtest/gtest.h>

TEST(ManeuverDetector, InflatesQOnlyAfterPersistentInnovation){
  aurora::estimation::ManeuverDetector detector(3,7.815,4,10.0);
  EXPECT_FALSE(detector.observe(2.0));
  EXPECT_FALSE(detector.observe(3.0));
  EXPECT_FALSE(detector.observe(4.0));
  EXPECT_DOUBLE_EQ(detector.q_scale(),1.0);
  EXPECT_FALSE(detector.observe(15.0));
  EXPECT_TRUE(detector.observe(16.0));
  EXPECT_DOUBLE_EQ(detector.q_scale(),10.0);
}
