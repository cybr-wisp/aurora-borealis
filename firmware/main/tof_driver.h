#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  bool fake_backend;
  uint64_t fake_sequence;
} aurora_tof_t;

esp_err_t aurora_tof_init(
    aurora_tof_t* tof,
    i2c_master_bus_handle_t bus,
    bool fake_backend);

esp_err_t aurora_tof_read_m(
    aurora_tof_t* tof,
    float* distance_m);
