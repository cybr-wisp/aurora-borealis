#include "association/track_manager.h"

#include <algorithm>
#include <stdexcept>

namespace aurora::association {

TrackManager::TrackManager(
    TrackManagerConfig config)
    : config_(config) {
  if (config_.confirmation_hits < 1 ||
      config_.coast_after_misses < 1 ||
      config_.tentative_delete_misses < 1 ||
      config_.delete_after_misses <
          config_.coast_after_misses) {
    throw std::invalid_argument(
        "invalid track lifecycle configuration");
  }
}

std::uint64_t TrackManager::create(
    const estimation::StateVector& state,
    const estimation::Covariance& covariance) {
  Track track;
  track.id = next_id_++;
  track.state = state;
  track.covariance = covariance;
  track.status = TrackStatus::kTentative;
  track.consecutive_hits = 1;
  track.consecutive_misses = 0;
  track.total_hits = 1;
  track.age_frames = 1;

  if (config_.confirmation_hits == 1) {
    track.status = TrackStatus::kConfirmed;
    track.ever_confirmed = true;
  }

  tracks_.push_back(track);
  return track.id;
}

void TrackManager::update_estimate(
    std::uint64_t track_id,
    const estimation::StateVector& state,
    const estimation::Covariance& covariance) {
  auto* track = find(track_id);
  if (track == nullptr) {
    throw std::out_of_range("unknown track id");
  }

  track->state = state;
  track->covariance = covariance;
}

void TrackManager::mark_associated(
    std::uint64_t track_id,
    const estimation::StateVector& state,
    const estimation::Covariance& covariance) {
  auto* track = find(track_id);
  if (track == nullptr) {
    throw std::out_of_range("unknown track id");
  }

  track->state = state;
  track->covariance = covariance;
  ++track->age_frames;
  ++track->total_hits;
  ++track->consecutive_hits;
  track->consecutive_misses = 0;

  if (track->ever_confirmed ||
      track->consecutive_hits >=
          config_.confirmation_hits) {
    track->status = TrackStatus::kConfirmed;
    track->ever_confirmed = true;
  } else {
    track->status = TrackStatus::kTentative;
  }
}

void TrackManager::mark_missed(
    std::uint64_t track_id) {
  auto* track = find(track_id);
  if (track == nullptr) {
    throw std::out_of_range("unknown track id");
  }

  ++track->age_frames;
  ++track->consecutive_misses;
  track->consecutive_hits = 0;

  if (!track->ever_confirmed) {
    if (track->consecutive_misses >=
        config_.tentative_delete_misses) {
      track->status = TrackStatus::kDeleted;
    }
    return;
  }

  if (track->consecutive_misses >=
      config_.delete_after_misses) {
    track->status = TrackStatus::kDeleted;
  } else if (track->consecutive_misses >=
             config_.coast_after_misses) {
    track->status = TrackStatus::kCoasting;
  }
}

Track* TrackManager::find(
    std::uint64_t track_id) {
  const auto it = std::find_if(
      tracks_.begin(),
      tracks_.end(),
      [track_id](const Track& track) {
        return track.id == track_id;
      });

  return it == tracks_.end() ? nullptr : &(*it);
}

const Track* TrackManager::find(
    std::uint64_t track_id) const {
  const auto it = std::find_if(
      tracks_.begin(),
      tracks_.end(),
      [track_id](const Track& track) {
        return track.id == track_id;
      });

  return it == tracks_.end() ? nullptr : &(*it);
}

std::vector<std::uint64_t>
TrackManager::active_track_ids() const {
  std::vector<std::uint64_t> ids;
  ids.reserve(tracks_.size());

  for (const auto& track : tracks_) {
    if (track.status != TrackStatus::kDeleted) {
      ids.push_back(track.id);
    }
  }

  return ids;
}

void TrackManager::prune_deleted() {
  tracks_.erase(
      std::remove_if(
          tracks_.begin(),
          tracks_.end(),
          [](const Track& track) {
            return track.status == TrackStatus::kDeleted;
          }),
      tracks_.end());
}

const char* to_string(
    TrackStatus status) noexcept {
  switch (status) {
    case TrackStatus::kTentative:
      return "TENTATIVE";
    case TrackStatus::kConfirmed:
      return "CONFIRMED";
    case TrackStatus::kCoasting:
      return "COASTING";
    case TrackStatus::kDeleted:
      return "DELETED";
  }
  return "UNKNOWN";
}

}  // namespace aurora::association
