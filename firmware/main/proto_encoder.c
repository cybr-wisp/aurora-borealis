#include "proto_encoder.h"

#include <stdbool.h>
#include <string.h>

static bool write_byte(
    uint8_t byte,
    uint8_t* output,
    size_t capacity,
    size_t* cursor) {
  if (*cursor >= capacity) {
    return false;
  }

  output[(*cursor)++] = byte;
  return true;
}

static bool write_varint(
    uint64_t value,
    uint8_t* output,
    size_t capacity,
    size_t* cursor) {
  while (value >= 0x80U) {
    if (!write_byte(
            (uint8_t)((value & 0x7FU) | 0x80U),
            output,
            capacity,
            cursor)) {
      return false;
    }
    value >>= 7U;
  }

  return write_byte(
      (uint8_t)value,
      output,
      capacity,
      cursor);
}

static bool write_fixed64_double(
    uint8_t key,
    double value,
    uint8_t* output,
    size_t capacity,
    size_t* cursor) {
  if (!write_byte(key, output, capacity, cursor)) {
    return false;
  }

  if (*cursor + sizeof(double) > capacity) {
    return false;
  }

  uint64_t bits = 0;
  memcpy(&bits, &value, sizeof(bits));

  for (int i = 0; i < 8; ++i) {
    output[(*cursor)++] =
        (uint8_t)((bits >> (8 * i)) & 0xFFU);
  }

  return true;
}

size_t aurora_encode_observation(
    const aurora_observation_wire_t* observation,
    uint8_t* output,
    size_t output_capacity) {
  size_t cursor = 0;

  if (!write_varint(8, output, output_capacity, &cursor) ||
      !write_varint(
          observation->sensor_id,
          output,
          output_capacity,
          &cursor) ||
      !write_varint(16, output, output_capacity, &cursor) ||
      !write_varint(
          observation->sequence_number,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          25,
          observation->timestamp_sec,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          33,
          observation->range_m,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          41,
          observation->azimuth_rad,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          49,
          observation->elevation_rad,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          57,
          observation->range_sigma,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          65,
          observation->azimuth_sigma,
          output,
          output_capacity,
          &cursor) ||
      !write_fixed64_double(
          73,
          observation->elevation_sigma,
          output,
          output_capacity,
          &cursor) ||
      !write_varint(80, output, output_capacity, &cursor) ||
      !write_varint(
          observation->sent_monotonic_ns,
          output,
          output_capacity,
          &cursor)) {
    return 0;
  }

  return cursor;
}
