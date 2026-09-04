#pragma once
#include <Eigen/Dense>

namespace aurora::estimation {
using StateVector = Eigen::Matrix<double, 6, 1>;
using Covariance = Eigen::Matrix<double, 6, 6>;
using MeasurementVector = Eigen::Matrix<double, 3, 1>;
using MeasurementCovariance = Eigen::Matrix<double, 3, 3>;
using MeasurementJacobian = Eigen::Matrix<double, 3, 6>;
}
