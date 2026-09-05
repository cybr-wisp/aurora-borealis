#include "ring_buffer.h"

#include <string.h>

void aurora_sensor_ring_init(
    aurora_sensor_ring_t* ring) {
  memset(ring, 0, sizeof(*ring));
  portMUX_TYPE unlocked = portMUX_INITIALIZER_UNLOCKED;
  ring->mux = unlocked;
}

void aurora_sensor_ring_push_latest(
    aurora_sensor_ring_t* ring,
    const aurora_sensor_sample_t* sample) {
  portENTER_CRITICAL(&ring->mux);

  if (ring->count == AURORA_SENSOR_RING_CAPACITY) {
    ring->tail =
        (ring->tail + 1U) %
        AURORA_SENSOR_RING_CAPACITY;
    --ring->count;
    ++ring->dropped_oldest;
  }

  ring->slots[ring->head] = *sample;
  ring->head =
      (ring->head + 1U) %
      AURORA_SENSOR_RING_CAPACITY;
  ++ring->count;

  portEXIT_CRITICAL(&ring->mux);
}

bool aurora_sensor_ring_pop(
    aurora_sensor_ring_t* ring,
    aurora_sensor_sample_t* sample) {
  bool available = false;

  portENTER_CRITICAL(&ring->mux);

  if (ring->count > 0U) {
    *sample = ring->slots[ring->tail];
    ring->tail =
        (ring->tail + 1U) %
        AURORA_SENSOR_RING_CAPACITY;
    --ring->count;
    available = true;
  }

  portEXIT_CRITICAL(&ring->mux);
  return available;
}

size_t aurora_sensor_ring_size(
    aurora_sensor_ring_t* ring) {
  size_t value;

  portENTER_CRITICAL(&ring->mux);
  value = ring->count;
  portEXIT_CRITICAL(&ring->mux);

  return value;
}

uint64_t aurora_sensor_ring_dropped(
    aurora_sensor_ring_t* ring) {
  uint64_t value;

  portENTER_CRITICAL(&ring->mux);
  value = ring->dropped_oldest;
  portEXIT_CRITICAL(&ring->mux);

  return value;
}
