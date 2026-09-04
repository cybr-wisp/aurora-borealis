#include "estimation/consistency.h"
#include <stdexcept>
namespace aurora::estimation {
double nees(const StateVector& truth, const StateVector& estimate, const Covariance& p) {
  const StateVector e = truth-estimate;
  Eigen::LDLT<Covariance> ldlt(p);
  if (ldlt.info()!=Eigen::Success) throw std::runtime_error("NEES covariance factorization failed");
  return e.dot(ldlt.solve(e));
}
}
