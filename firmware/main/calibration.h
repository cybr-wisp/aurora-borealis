#pragma once

#include "imu_driver.h"

typedef struct {
  float accel_bias_mps2[3];
  float gyro_bias_rps[3];
} aurora_imu_calibration_t;

void aurora_calibration_reset(
    aurora_imu_calibration_t* calibration);

void aurora_calibration_apply(
    const aurora_imu_calibration_t* calibration,
    aurora_imu_sample_t* sample);

esp_err_t aurora_calibrate_stationary(
    aurora_imu_t* imu,
    aurora_imu_calibration_t* calibration,
    int samples);
