#include "ingestion/validator.h"
#include <gtest/gtest.h>

namespace {
aurora::proto::Observation valid_obs(std::uint64_t seq,double timestamp){
  aurora::proto::Observation o; o.set_sensor_id(1); o.set_sequence_number(seq); o.set_timestamp_sec(timestamp);
  o.set_range_m(1000); o.set_range_sigma(10); o.set_azimuth_sigma(.003); o.set_elevation_sigma(.003); return o;
}
}
TEST(Validator, RejectsDuplicateAndStale){
  using namespace aurora::ingestion; Validator v(1.0);
  EXPECT_EQ(v.validate(valid_obs(1,10.0),10.2),ValidationResult::kAccept);
  EXPECT_EQ(v.validate(valid_obs(1,10.1),10.2),ValidationResult::kDuplicateOrOldSequence);
  EXPECT_EQ(v.validate(valid_obs(2,8.0),10.2),ValidationResult::kStale);
}
