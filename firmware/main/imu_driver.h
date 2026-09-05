#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
  float accel_mps2[3];
  float gyro_rps[3];
} aurora_imu_sample_t;

typedef struct {
  i2c_master_dev_handle_t device;
} aurora_imu_t;

esp_err_t aurora_imu_init(
    aurora_imu_t* imu,
    i2c_master_bus_handle_t bus);

esp_err_t aurora_imu_read(
    aurora_imu_t* imu,
    aurora_imu_sample_t* sample);

void aurora_imu_deinit(aurora_imu_t* imu);
