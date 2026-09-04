#pragma once
#include <Eigen/Dense>

namespace aurora::geometry {
struct Geodetic { double lat_rad; double lon_rad; double alt_m; };
using Ecef = Eigen::Vector3d;
Ecef geodetic_to_ecef(const Geodetic& g);
Geodetic ecef_to_geodetic(const Ecef& ecef);
}
