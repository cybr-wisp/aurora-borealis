#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint32_t sensor_id;
  uint64_t sequence_number;
  double timestamp_sec;
  double range_m;
  double azimuth_rad;
  double elevation_rad;
  double range_sigma;
  double azimuth_sigma;
  double elevation_sigma;
  uint64_t sent_monotonic_ns;
} aurora_observation_wire_t;

// Encodes the exact wire schema in engine/proto/observation.proto.
// Returns 0 if output_capacity is insufficient.
size_t aurora_encode_observation(
    const aurora_observation_wire_t* observation,
    uint8_t* output,
    size_t output_capacity);
