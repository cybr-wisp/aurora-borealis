#pragma once
#include <Eigen/Dense>

namespace aurora::geometry {
Eigen::Vector3d spherical_to_cartesian(double range_m, double azimuth_rad, double elevation_rad);
Eigen::Vector3d cartesian_to_spherical(const Eigen::Vector3d& relative_enu_m);
double wrap_angle(double rad);
}
