#include "association/hungarian.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace aurora::association {

std::vector<int> hungarian_assign(
    const Eigen::MatrixXd& cost_matrix,
    double max_cost) {
  if (!(max_cost > 0.0) || !std::isfinite(max_cost)) {
    throw std::invalid_argument(
        "max_cost must be finite and positive");
  }

  const int rows = static_cast<int>(cost_matrix.rows());
  const int cols = static_cast<int>(cost_matrix.cols());

  if (rows == 0) {
    return {};
  }

  if (cols == 0) {
    return std::vector<int>(
        static_cast<std::size_t>(rows),
        -1);
  }

  const int n = std::max(rows, cols);
  const double dummy_cost = max_cost;
  const double forbidden_cost = max_cost + 1.0;

  std::vector<std::vector<double>> a(
      static_cast<std::size_t>(n),
      std::vector<double>(
          static_cast<std::size_t>(n),
          dummy_cost));

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      const double raw = cost_matrix(i, j);
      a[static_cast<std::size_t>(i)]
       [static_cast<std::size_t>(j)] =
          std::isfinite(raw) && raw <= max_cost
              ? raw
              : forbidden_cost;
    }
  }

  // Shortest augmenting-path Hungarian algorithm, 1-indexed.
  std::vector<double> u(
      static_cast<std::size_t>(n + 1),
      0.0);
  std::vector<double> v(
      static_cast<std::size_t>(n + 1),
      0.0);
  std::vector<int> p(
      static_cast<std::size_t>(n + 1),
      0);
  std::vector<int> way(
      static_cast<std::size_t>(n + 1),
      0);

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;

    std::vector<double> minv(
        static_cast<std::size_t>(n + 1),
        std::numeric_limits<double>::infinity());
    std::vector<bool> used(
        static_cast<std::size_t>(n + 1),
        false);

    do {
      used[static_cast<std::size_t>(j0)] = true;
      const int i0 = p[static_cast<std::size_t>(j0)];

      double delta =
          std::numeric_limits<double>::infinity();
      int j1 = 0;

      for (int j = 1; j <= n; ++j) {
        if (used[static_cast<std::size_t>(j)]) {
          continue;
        }

        const double cur =
            a[static_cast<std::size_t>(i0 - 1)]
             [static_cast<std::size_t>(j - 1)] -
            u[static_cast<std::size_t>(i0)] -
            v[static_cast<std::size_t>(j)];

        if (cur <
            minv[static_cast<std::size_t>(j)]) {
          minv[static_cast<std::size_t>(j)] = cur;
          way[static_cast<std::size_t>(j)] = j0;
        }

        if (minv[static_cast<std::size_t>(j)] < delta) {
          delta =
              minv[static_cast<std::size_t>(j)];
          j1 = j;
        }
      }

      for (int j = 0; j <= n; ++j) {
        if (used[static_cast<std::size_t>(j)]) {
          u[static_cast<std::size_t>(
              p[static_cast<std::size_t>(j)])] += delta;
          v[static_cast<std::size_t>(j)] -= delta;
        } else {
          minv[static_cast<std::size_t>(j)] -= delta;
        }
      }

      j0 = j1;
    } while (p[static_cast<std::size_t>(j0)] != 0);

    do {
      const int j1 =
          way[static_cast<std::size_t>(j0)];
      p[static_cast<std::size_t>(j0)] =
          p[static_cast<std::size_t>(j1)];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(
      static_cast<std::size_t>(rows),
      -1);

  for (int j = 1; j <= n; ++j) {
    const int assigned_row =
        p[static_cast<std::size_t>(j)] - 1;
    const int assigned_col = j - 1;

    if (assigned_row < 0 ||
        assigned_row >= rows ||
        assigned_col < 0 ||
        assigned_col >= cols) {
      continue;
    }

    const double original =
        cost_matrix(assigned_row, assigned_col);

    if (std::isfinite(original) &&
        original <= max_cost) {
      assignment[
          static_cast<std::size_t>(assigned_row)] =
          assigned_col;
    }
  }

  return assignment;
}

}  // namespace aurora::association
