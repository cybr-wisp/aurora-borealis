#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define AURORA_SENSOR_RING_CAPACITY 256

typedef struct {
  uint64_t sequence_number;
  double timestamp_sec;
  float accel_mps2[3];
  float gyro_rps[3];
  float distance_m;
  int32_t sample_interval_us;
} aurora_sensor_sample_t;

typedef struct {
  aurora_sensor_sample_t slots[AURORA_SENSOR_RING_CAPACITY];
  size_t head;
  size_t tail;
  size_t count;
  uint64_t dropped_oldest;
  portMUX_TYPE mux;
} aurora_sensor_ring_t;

void aurora_sensor_ring_init(aurora_sensor_ring_t* ring);

void aurora_sensor_ring_push_latest(
    aurora_sensor_ring_t* ring,
    const aurora_sensor_sample_t* sample);

bool aurora_sensor_ring_pop(
    aurora_sensor_ring_t* ring,
    aurora_sensor_sample_t* sample);

size_t aurora_sensor_ring_size(
    aurora_sensor_ring_t* ring);

uint64_t aurora_sensor_ring_dropped(
    aurora_sensor_ring_t* ring);
