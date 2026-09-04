#pragma once
#include "estimation/state.h"
#include <utility>

namespace aurora::estimation {
double nees(const StateVector& truth, const StateVector& estimate, const Covariance& covariance);
constexpr std::pair<double,double> nees_95_bounds_6d() { return {1.2373442458, 14.4493753354}; }
}
