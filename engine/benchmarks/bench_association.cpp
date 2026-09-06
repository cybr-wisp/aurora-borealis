#include "association/hungarian.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

double percentile(
    const std::vector<double>& sorted,
    double p) {
  const auto index = static_cast<std::size_t>(
      p * static_cast<double>(sorted.size() - 1));
  return sorted[index];
}

void run_case(int tracks, int iterations) {
  std::mt19937_64 rng(
      static_cast<std::uint64_t>(20260905 + tracks));
  std::normal_distribution<double> noise(0.0, 0.15);

  std::vector<double> latency_us;
  latency_us.reserve(static_cast<std::size_t>(iterations));

  for (int iteration = 0; iteration < iterations; ++iteration) {
    Eigen::MatrixXd costs(tracks, tracks);

    for (int row = 0; row < tracks; ++row) {
      for (int col = 0; col < tracks; ++col) {
        const double separation =
            static_cast<double>(std::abs(row - col));
        costs(row, col) =
            0.25 + 2.0 * separation + std::abs(noise(rng));
      }
    }

    const auto started = Clock::now();
    const auto assignment =
        aurora::association::hungarian_assign(costs, 50.0);
    const auto ended = Clock::now();

    if (assignment.size() != static_cast<std::size_t>(tracks)) {
      std::exit(2);
    }

    latency_us.push_back(
        std::chrono::duration<double, std::micro>(
            ended - started).count());
  }

  std::sort(latency_us.begin(), latency_us.end());

  std::cout
      << tracks << ","
      << iterations << ","
      << percentile(latency_us, 0.50) << ","
      << percentile(latency_us, 0.95) << ","
      << percentile(latency_us, 0.99)
      << "\n";
}

}  // namespace

int main() {
  std::cout << std::fixed << std::setprecision(3);
  std::cout
      << "tracks,iterations,p50_us,p95_us,p99_us\n";

  // Day 9 target-count points.
  run_case(1, 700);
  run_case(3, 600);
  run_case(5, 500);
  run_case(10, 300);

  // Day 5 scaling points retained for the wider failure/performance envelope.
  run_case(25, 200);
  run_case(50, 100);
  run_case(100, 30);

  return 0;
}
