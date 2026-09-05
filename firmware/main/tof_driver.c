#include "tof_driver.h"

#include "esp_log.h"

#include <math.h>

static const char* TAG = "aurora_tof";

esp_err_t aurora_tof_init(
    aurora_tof_t* tof,
    i2c_master_bus_handle_t bus,
    bool fake_backend) {
  (void)bus;

  tof->fake_backend = fake_backend;
  tof->fake_sequence = 0;

  if (!fake_backend) {
    // The VL53L1X startup/calibration sequence should come from ST's
    // Ultra Lite Driver rather than a guessed register table. The adapter
    // boundary is intentionally ready now; hardware validation supplies
    // the concrete ULD component in Day 6B.
    ESP_LOGW(
        TAG,
        "real VL53L1X backend pending ST ULD integration");
    return ESP_ERR_NOT_SUPPORTED;
  }

  return ESP_OK;
}

esp_err_t aurora_tof_read_m(
    aurora_tof_t* tof,
    float* distance_m) {
  if (!tof->fake_backend) {
    return ESP_ERR_NOT_SUPPORTED;
  }

  const double phase =
      (double)tof->fake_sequence * 0.02;
  *distance_m =
      (float)(2.0 + 0.25 * sin(phase));

  ++tof->fake_sequence;
  return ESP_OK;
}
