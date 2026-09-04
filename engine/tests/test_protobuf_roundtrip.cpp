#include "observation.pb.h"
#include <gtest/gtest.h>

TEST(Protobuf, ObservationRoundTrip){
  aurora::proto::Observation a;
  a.set_sensor_id(9); a.set_sequence_number(123); a.set_timestamp_sec(42.25);
  a.set_range_m(1200.0); a.set_azimuth_rad(0.4); a.set_elevation_rad(0.1);
  a.set_range_sigma(10.0); a.set_azimuth_sigma(0.003); a.set_elevation_sigma(0.003);
  std::string bytes; ASSERT_TRUE(a.SerializeToString(&bytes));
  aurora::proto::Observation b; ASSERT_TRUE(b.ParseFromString(bytes));
  EXPECT_EQ(b.sensor_id(),9u); EXPECT_EQ(b.sequence_number(),123u); EXPECT_DOUBLE_EQ(b.timestamp_sec(),42.25);
}
