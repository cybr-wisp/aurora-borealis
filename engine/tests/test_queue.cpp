#include "ingestion/observation_queue.h"

#include <gtest/gtest.h>

TEST(SpscRing, BoundedOverflowIsExplicit) {
  aurora::ingestion::SpscRing<int, 4> q;

  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  EXPECT_TRUE(q.push(3));
  EXPECT_FALSE(q.push(4));

  EXPECT_EQ(q.size_approx(), 3U);
  EXPECT_EQ(q.high_water_mark(), 3U);

  EXPECT_EQ(q.pop().value(), 1);
  EXPECT_TRUE(q.push(4));
}

TEST(SpscRing, ConsumerCanShedOldestBacklog) {
  aurora::ingestion::SpscRing<int, 8> q;

  for (int value = 1; value <= 6; ++value) {
    ASSERT_TRUE(q.push(value));
  }

  EXPECT_EQ(q.discard_oldest(4), 4U);
  ASSERT_EQ(q.size_approx(), 2U);

  EXPECT_EQ(q.pop().value(), 5);
  EXPECT_EQ(q.pop().value(), 6);
  EXPECT_FALSE(q.pop().has_value());
}

TEST(SpscRing, ReportsUsableCapacity) {
  using Queue = aurora::ingestion::SpscRing<int, 16>;
  EXPECT_EQ(Queue::usable_capacity(), 15U);
}
