#include "estimation/consistency.h"
#include <gtest/gtest.h>
TEST(Consistency, NeesZeroForPerfectEstimate){
  using namespace aurora::estimation; StateVector x=StateVector::Zero(); Covariance p=Covariance::Identity(); EXPECT_DOUBLE_EQ(nees(x,x,p),0.0);
}
