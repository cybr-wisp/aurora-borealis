#include "geometry/enu.h"
#include <cmath>

namespace aurora::geometry {
EnuFrame::EnuFrame(Geodetic ref) : origin_(geodetic_to_ecef(ref)) {
  const double slat = std::sin(ref.lat_rad), clat = std::cos(ref.lat_rad);
  const double slon = std::sin(ref.lon_rad), clon = std::cos(ref.lon_rad);
  r_ecef_to_enu_ << -slon, clon, 0.0,
                    -slat * clon, -slat * slon, clat,
                     clat * clon,  clat * slon, slat;
}
Eigen::Vector3d EnuFrame::ecef_to_enu(const Ecef& ecef) const { return r_ecef_to_enu_ * (ecef - origin_); }
Ecef EnuFrame::enu_to_ecef(const Eigen::Vector3d& enu) const { return origin_ + r_ecef_to_enu_.transpose() * enu; }
}
