#include "calibration.h"

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

void aurora_calibration_reset(
    aurora_imu_calibration_t* calibration) {
  memset(calibration, 0, sizeof(*calibration));
}

void aurora_calibration_apply(
    const aurora_imu_calibration_t* calibration,
    aurora_imu_sample_t* sample) {
  for (int axis = 0; axis < 3; ++axis) {
    sample->accel_mps2[axis] -=
        calibration->accel_bias_mps2[axis];
    sample->gyro_rps[axis] -=
        calibration->gyro_bias_rps[axis];
  }
}

esp_err_t aurora_calibrate_stationary(
    aurora_imu_t* imu,
    aurora_imu_calibration_t* calibration,
    int samples) {
  if (samples <= 0) {
    return ESP_ERR_INVALID_ARG;
  }

  aurora_calibration_reset(calibration);

  for (int i = 0; i < samples; ++i) {
    aurora_imu_sample_t sample;

    ESP_RETURN_ON_ERROR(
        aurora_imu_read(imu, &sample),
        "aurora_calibration",
        "IMU calibration read");

    for (int axis = 0; axis < 3; ++axis) {
      calibration->accel_bias_mps2[axis] +=
          sample.accel_mps2[axis];
      calibration->gyro_bias_rps[axis] +=
          sample.gyro_rps[axis];
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }

  for (int axis = 0; axis < 3; ++axis) {
    calibration->accel_bias_mps2[axis] /=
        (float)samples;
    calibration->gyro_bias_rps[axis] /=
        (float)samples;
  }

  // Startup calibration assumes the board is stationary and +Z faces up.
  calibration->accel_bias_mps2[2] -= 9.80665f;

  return ESP_OK;
}
