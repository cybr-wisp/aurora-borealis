#pragma once

#include <Eigen/Dense>

#include <vector>

namespace aurora::association {

// Returns one measurement column per track row. -1 means unassigned.
// Costs greater than max_cost or non-finite costs are forbidden.
std::vector<int> hungarian_assign(
    const Eigen::MatrixXd& cost_matrix,
    double max_cost);

}  // namespace aurora::association
