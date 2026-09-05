#include "association/track_manager.h"

#include <gtest/gtest.h>

TEST(TrackManager, TentativeConfirmedCoastingDeletedLifecycle) {
  aurora::association::TrackManager manager({
      3, 1, 2, 3});

  const auto x =
      aurora::estimation::StateVector::Zero();
  const auto p =
      aurora::estimation::Covariance::Identity();

  const auto id = manager.create(x, p);

  ASSERT_NE(manager.find(id), nullptr);
  EXPECT_EQ(
      manager.find(id)->status,
      aurora::association::TrackStatus::kTentative);

  manager.mark_associated(id, x, p);
  EXPECT_EQ(
      manager.find(id)->status,
      aurora::association::TrackStatus::kTentative);

  manager.mark_associated(id, x, p);
  EXPECT_EQ(
      manager.find(id)->status,
      aurora::association::TrackStatus::kConfirmed);

  manager.mark_missed(id);
  EXPECT_EQ(
      manager.find(id)->status,
      aurora::association::TrackStatus::kCoasting);

  manager.mark_missed(id);
  EXPECT_EQ(
      manager.find(id)->status,
      aurora::association::TrackStatus::kCoasting);

  manager.mark_missed(id);
  EXPECT_EQ(
      manager.find(id)->status,
      aurora::association::TrackStatus::kDeleted);

  manager.prune_deleted();
  EXPECT_EQ(manager.find(id), nullptr);
}
