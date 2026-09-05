#include "sensor_task.h"

#include "calibration.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "imu_driver.h"
#include "time_sync.h"
#include "tof_driver.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdlib.h>

static const char* TAG = "aurora_sensor";

typedef struct {
  aurora_sensor_task_config_t config;
} task_context_t;

static void sensor_task(void* raw_context) {
  task_context_t* context =
      (task_context_t*)raw_context;

  aurora_imu_t imu = {0};
  aurora_tof_t tof = {0};
  aurora_imu_calibration_t calibration;

  aurora_calibration_reset(&calibration);

  if (!context->config.fake_sensors) {
    if (aurora_imu_init(
            &imu,
            context->config.i2c_bus) != ESP_OK) {
      ESP_LOGE(TAG, "MPU6050 initialization failed");
      vTaskDelete(NULL);
      return;
    }

    if (aurora_calibrate_stationary(
            &imu,
            &calibration,
            200) != ESP_OK) {
      ESP_LOGE(TAG, "IMU calibration failed");
      vTaskDelete(NULL);
      return;
    }
  }

  if (aurora_tof_init(
          &tof,
          context->config.i2c_bus,
          context->config.fake_sensors) != ESP_OK) {
    ESP_LOGE(
        TAG,
        "ToF backend unavailable; use fake backend until Day 6B hardware integration");
    vTaskDelete(NULL);
    return;
  }

  TickType_t wake = xTaskGetTickCount();
  int64_t previous_sample_us = esp_timer_get_time();
  int64_t max_abs_jitter_us = 0;
  uint64_t sequence = 0;

  for (;;) {
    vTaskDelayUntil(
        &wake,
        pdMS_TO_TICKS(10));

    const int64_t sample_us =
        esp_timer_get_time();

    const int32_t interval_us =
        (int32_t)(sample_us - previous_sample_us);
    previous_sample_us = sample_us;

    const int64_t jitter_us =
        llabs((long long)interval_us - 10000LL);

    if (jitter_us > max_abs_jitter_us) {
      max_abs_jitter_us = jitter_us;
    }

    aurora_imu_sample_t imu_sample = {0};
    float distance_m = 0.0f;

    if (context->config.fake_sensors) {
      const double phase =
          (double)sequence * 0.015;

      imu_sample.accel_mps2[0] =
          (float)(0.1 * sin(phase));
      imu_sample.accel_mps2[1] =
          (float)(0.1 * cos(phase));
      imu_sample.accel_mps2[2] = 9.80665f;

      imu_sample.gyro_rps[2] =
          (float)(0.02 * sin(phase * 0.5));
    } else {
      if (aurora_imu_read(
              &imu,
              &imu_sample) != ESP_OK) {
        ESP_LOGW(TAG, "IMU read failed");
        continue;
      }

      aurora_calibration_apply(
          &calibration,
          &imu_sample);
    }

    if (aurora_tof_read_m(
            &tof,
            &distance_m) != ESP_OK) {
      ESP_LOGW(TAG, "ToF read failed");
      continue;
    }

    aurora_sensor_sample_t sample = {
        .sequence_number = sequence,
        .timestamp_sec = aurora_epoch_sec(),
        .accel_mps2 = {
            imu_sample.accel_mps2[0],
            imu_sample.accel_mps2[1],
            imu_sample.accel_mps2[2]},
        .gyro_rps = {
            imu_sample.gyro_rps[0],
            imu_sample.gyro_rps[1],
            imu_sample.gyro_rps[2]},
        .distance_m = distance_m,
        .sample_interval_us = interval_us,
    };

    aurora_sensor_ring_push_latest(
        context->config.ring,
        &sample);

    ++sequence;

    if (sequence % 500U == 0U) {
      ESP_LOGI(
          TAG,
          "samples=%llu max_abs_jitter_us=%lld ring_depth=%u dropped_oldest=%llu",
          (unsigned long long)sequence,
          (long long)max_abs_jitter_us,
          (unsigned)aurora_sensor_ring_size(
              context->config.ring),
          (unsigned long long)aurora_sensor_ring_dropped(
              context->config.ring));

      max_abs_jitter_us = 0;
    }
  }
}

esp_err_t aurora_sensor_task_start(
    const aurora_sensor_task_config_t* config) {
  task_context_t* context =
      calloc(1, sizeof(*context));

  if (context == NULL) {
    return ESP_ERR_NO_MEM;
  }

  context->config = *config;

  const BaseType_t result =
      xTaskCreatePinnedToCore(
          sensor_task,
          "SensorTask",
          4096,
          context,
          8,
          NULL,
          1);

  if (result != pdPASS) {
    free(context);
    return ESP_FAIL;
  }

  return ESP_OK;
}
