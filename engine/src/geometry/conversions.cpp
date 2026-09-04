#include "geometry/conversions.h"
#include <cmath>
#include <stdexcept>
#include <numbers>

namespace aurora::geometry {
Eigen::Vector3d spherical_to_cartesian(double r, double az, double el) {
  const double ce = std::cos(el);
  return {r * ce * std::cos(az), r * ce * std::sin(az), r * std::sin(el)};
}
Eigen::Vector3d cartesian_to_spherical(const Eigen::Vector3d& p) {
  const double r = p.norm();
  if (r < 1e-12) throw std::invalid_argument("zero relative vector");
  return {r, std::atan2(p.y(), p.x()), std::atan2(p.z(), std::hypot(p.x(), p.y()))};
}
double wrap_angle(double a) {
  while (a > std::numbers::pi) a -= 2.0 * std::numbers::pi;
  while (a < -std::numbers::pi) a += 2.0 * std::numbers::pi;
  return a;
}
}
