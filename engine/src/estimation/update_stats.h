#pragma once

#include "estimation/state.h"

namespace aurora::estimation {

struct UpdateStats {
    MeasurementVector innovation;
    MeasurementCovariance innovation_covariance;
    double nis;
};

}  // namespace aurora::estimation
