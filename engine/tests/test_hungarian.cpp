#include "association/hungarian.h"

#include <gtest/gtest.h>

TEST(Hungarian, FindsMinimumCostOneToOneAssignment) {
  Eigen::MatrixXd costs(3, 3);
  costs <<
      1.0, 9.0, 9.0,
      9.0, 2.0, 9.0,
      9.0, 9.0, 3.0;

  const auto assignment =
      aurora::association::hungarian_assign(
          costs,
          10.0);

  ASSERT_EQ(assignment.size(), 3U);
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_EQ(assignment[2], 2);
}

TEST(Hungarian, LeavesForbiddenRowUnassigned) {
  Eigen::MatrixXd costs(2, 2);
  costs <<
      1.0, 50.0,
      50.0, 50.0;

  const auto assignment =
      aurora::association::hungarian_assign(
          costs,
          10.0);

  ASSERT_EQ(assignment.size(), 2U);
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], -1);
}

TEST(Hungarian, SupportsRectangularMatrices) {
  Eigen::MatrixXd costs(3, 2);
  costs <<
      1.0, 8.0,
      8.0, 1.0,
      9.0, 9.0;

  const auto assignment =
      aurora::association::hungarian_assign(
          costs,
          5.0);

  ASSERT_EQ(assignment.size(), 3U);
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_EQ(assignment[2], -1);
}
