#pragma once
#include <deque>
#include <cstddef>

namespace aurora::estimation {
class ManeuverDetector {
 public:
  explicit ManeuverDetector(std::size_t window = 5, double mean_nis_threshold = 7.815, std::size_t hold_steps = 12, double inflation = 10.0);
  bool observe(double nis);
  double q_scale() const { return active_ ? inflation_ : 1.0; }
 private:
  std::size_t window_, hold_steps_, remaining_=0;
  double threshold_, inflation_;
  bool active_=false;
  std::deque<double> values_;
};
}
