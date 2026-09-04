#include "geometry/wgs84.h"
#include "geometry/enu.h"
#include <gtest/gtest.h>
#include <cmath>
#include <numbers>

TEST(Coordinates, Wgs84RoundTripSubMillimeter){
  using namespace aurora::geometry;
  Geodetic g{45.4215*std::numbers::pi/180.0,-75.6972*std::numbers::pi/180.0,92.3};
  auto recovered=ecef_to_geodetic(geodetic_to_ecef(g));
  EXPECT_NEAR(recovered.lat_rad,g.lat_rad,1e-11);
  EXPECT_NEAR(recovered.lon_rad,g.lon_rad,1e-11);
  EXPECT_NEAR(recovered.alt_m,g.alt_m,1e-3);
}
TEST(Coordinates, EnuRoundTripSubMillimeter){
  using namespace aurora::geometry;
  Geodetic ref{45.4215*std::numbers::pi/180.0,-75.6972*std::numbers::pi/180.0,70.0}; EnuFrame frame(ref);
  Eigen::Vector3d enu(1234.567,-876.543,45.678);
  EXPECT_LT((frame.ecef_to_enu(frame.enu_to_ecef(enu))-enu).norm(),1e-3);
}
