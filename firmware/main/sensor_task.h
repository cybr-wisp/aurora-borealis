#pragma once

#include "ring_buffer.h"

#include "driver/i2c_master.h"
#include "esp_err.h"

#include <stdbool.h>

typedef struct {
  aurora_sensor_ring_t* ring;
  i2c_master_bus_handle_t i2c_bus;
  bool fake_sensors;
} aurora_sensor_task_config_t;

esp_err_t aurora_sensor_task_start(
    const aurora_sensor_task_config_t* config);
