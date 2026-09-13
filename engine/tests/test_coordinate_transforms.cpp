#include "geometry/wgs84.h"
#include "geometry/conversions.h"
#include "geometry/enu.h"
#include <gtest/gtest.h>
#include <cmath>
#include <numbers>
#include <limits>
#include <stdexcept>

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

TEST(Coordinates, WrapAngleHandlesLargeFiniteInput) {
  using namespace aurora::geometry;

  const double wrapped =
      wrap_angle(1.0e12 * std::numbers::pi + 0.25);

  EXPECT_TRUE(std::isfinite(wrapped));
  EXPECT_GE(wrapped, -std::numbers::pi);
  EXPECT_LE(wrapped, std::numbers::pi);

  EXPECT_NEAR(
      wrap_angle(0.25 + 4.0 * std::numbers::pi),
      0.25,
      1e-12);
}

TEST(Coordinates, WrapAngleRejectsNonFiniteInput) {
  using namespace aurora::geometry;

  EXPECT_THROW(
      wrap_angle(std::numeric_limits<double>::infinity()),
      std::invalid_argument);

  EXPECT_THROW(
      wrap_angle(std::numeric_limits<double>::quiet_NaN()),
      std::invalid_argument);
}