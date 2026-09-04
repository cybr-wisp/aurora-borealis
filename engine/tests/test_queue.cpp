#include "ingestion/observation_queue.h"
#include <gtest/gtest.h>
TEST(SpscRing, BoundedOverflowIsExplicit){
  aurora::ingestion::SpscRing<int,4> q;
  EXPECT_TRUE(q.push(1)); EXPECT_TRUE(q.push(2)); EXPECT_TRUE(q.push(3)); EXPECT_FALSE(q.push(4));
  EXPECT_EQ(q.pop().value(),1); EXPECT_TRUE(q.push(4));
}
