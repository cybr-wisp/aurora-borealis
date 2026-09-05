#pragma once

#include "ring_buffer.h"

#include "esp_err.h"

#include <stdint.h>

typedef struct {
  aurora_sensor_ring_t* ring;
  uint32_t sensor_id;
  const char* engine_ipv4;
  uint16_t engine_port;
} aurora_telemetry_task_config_t;

esp_err_t aurora_telemetry_task_start(
    const aurora_telemetry_task_config_t* config);
