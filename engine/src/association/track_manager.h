#pragma once

#include "estimation/state.h"

#include <cstdint>
#include <vector>

namespace aurora::association {

enum class TrackStatus {
  kTentative,
  kConfirmed,
  kCoasting,
  kDeleted
};

struct TrackManagerConfig {
  int confirmation_hits{3};
  int coast_after_misses{1};
  int tentative_delete_misses{2};
  int delete_after_misses{5};
};

struct Track {
  std::uint64_t id{0};
  estimation::StateVector state{
      estimation::StateVector::Zero()};
  estimation::Covariance covariance{
      estimation::Covariance::Identity()};
  TrackStatus status{TrackStatus::kTentative};

  int consecutive_hits{0};
  int consecutive_misses{0};
  std::uint64_t total_hits{0};
  std::uint64_t age_frames{0};
  bool ever_confirmed{false};
};

class TrackManager {
 public:
  explicit TrackManager(
      TrackManagerConfig config = {});

  std::uint64_t create(
      const estimation::StateVector& state,
      const estimation::Covariance& covariance);

  void update_estimate(
      std::uint64_t track_id,
      const estimation::StateVector& state,
      const estimation::Covariance& covariance);

  void mark_associated(
      std::uint64_t track_id,
      const estimation::StateVector& state,
      const estimation::Covariance& covariance);

  void mark_missed(std::uint64_t track_id);

  Track* find(std::uint64_t track_id);
  const Track* find(std::uint64_t track_id) const;

  [[nodiscard]]
  std::vector<std::uint64_t> active_track_ids() const;

  void prune_deleted();

  [[nodiscard]]
  const std::vector<Track>& tracks() const noexcept {
    return tracks_;
  }

 private:
  TrackManagerConfig config_;
  std::vector<Track> tracks_;
  std::uint64_t next_id_{1};
};

const char* to_string(TrackStatus status) noexcept;

}  // namespace aurora::association
