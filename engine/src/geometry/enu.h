#pragma once
#include "geometry/wgs84.h"
#include <Eigen/Dense>

namespace aurora::geometry {
class EnuFrame {
 public:
  explicit EnuFrame(Geodetic reference);
  Eigen::Vector3d ecef_to_enu(const Ecef& ecef) const;
  Ecef enu_to_ecef(const Eigen::Vector3d& enu) const;
  const Ecef& origin_ecef() const { return origin_; }
 private:
  Ecef origin_;
  Eigen::Matrix3d r_ecef_to_enu_;
};
}
