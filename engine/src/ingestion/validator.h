#pragma once
#include "observation.pb.h"
#include <unordered_map>
#include <cstdint>

namespace aurora::ingestion {
enum class ValidationResult { kAccept, kDuplicateOrOldSequence, kStale, kInvalid };
class Validator {
 public:
  explicit Validator(double max_staleness_sec = 1.0) : max_staleness_sec_(max_staleness_sec) {}
  ValidationResult validate(const aurora::proto::Observation& obs, double now_sec);
 private:
  double max_staleness_sec_;
  std::unordered_map<std::uint32_t, std::uint64_t> last_sequence_;
};
}
