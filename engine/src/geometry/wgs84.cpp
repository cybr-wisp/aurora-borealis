#include "geometry/wgs84.h"
#include <cmath>
#include <stdexcept>

namespace aurora::geometry {
namespace {
constexpr double kA = 6378137.0;
constexpr double kF = 1.0 / 298.257223563;
constexpr double kE2 = kF * (2.0 - kF);
}

Ecef geodetic_to_ecef(const Geodetic& g) {
  const double s = std::sin(g.lat_rad), c = std::cos(g.lat_rad);
  const double n = kA / std::sqrt(1.0 - kE2 * s * s);
  return {(n + g.alt_m) * c * std::cos(g.lon_rad),
          (n + g.alt_m) * c * std::sin(g.lon_rad),
          (n * (1.0 - kE2) + g.alt_m) * s};
}

Geodetic ecef_to_geodetic(const Ecef& p) {
  const double x = p.x(), y = p.y(), z = p.z();
  const double lon = std::atan2(y, x);
  const double rho = std::hypot(x, y);
  if (rho < 1e-9) throw std::invalid_argument("ECEF point too close to polar singularity");
  double lat = std::atan2(z, rho * (1.0 - kE2));
  double alt = 0.0;
  for (int i = 0; i < 12; ++i) {
    const double s = std::sin(lat);
    const double n = kA / std::sqrt(1.0 - kE2 * s * s);
    alt = rho / std::cos(lat) - n;
    const double next = std::atan2(z, rho * (1.0 - kE2 * n / (n + alt)));
    if (std::abs(next - lat) < 1e-14) { lat = next; break; }
    lat = next;
  }
  return {lat, lon, alt};
}
}
