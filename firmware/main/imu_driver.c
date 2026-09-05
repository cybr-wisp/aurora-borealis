#include "imu_driver.h"

#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <math.h>
#include <stdint.h>

#define MPU6050_ADDRESS 0x68
#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

static esp_err_t write_register(
    aurora_imu_t* imu,
    uint8_t reg,
    uint8_t value) {
  const uint8_t payload[2] = {reg, value};

  for (int attempt = 0; attempt < 4; ++attempt) {
    const esp_err_t err =
        i2c_master_transmit(
            imu->device,
            payload,
            sizeof(payload),
            50);

    if (err == ESP_OK) {
      return ESP_OK;
    }

    vTaskDelay(
        pdMS_TO_TICKS(1U << attempt));
  }

  return ESP_FAIL;
}

static esp_err_t read_registers(
    aurora_imu_t* imu,
    uint8_t start_reg,
    uint8_t* output,
    size_t length) {
  for (int attempt = 0; attempt < 4; ++attempt) {
    const esp_err_t err =
        i2c_master_transmit_receive(
            imu->device,
            &start_reg,
            1,
            output,
            length,
            50);

    if (err == ESP_OK) {
      return ESP_OK;
    }

    vTaskDelay(
        pdMS_TO_TICKS(1U << attempt));
  }

  return ESP_FAIL;
}

static int16_t be_i16(
    const uint8_t* bytes) {
  return (int16_t)(
      ((uint16_t)bytes[0] << 8U) |
      (uint16_t)bytes[1]);
}

esp_err_t aurora_imu_init(
    aurora_imu_t* imu,
    i2c_master_bus_handle_t bus) {
  const i2c_device_config_t config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_ADDRESS,
      .scl_speed_hz = 400000,
  };

  ESP_RETURN_ON_ERROR(
      i2c_master_bus_add_device(
          bus,
          &config,
          &imu->device),
      "aurora_imu",
      "add MPU6050 device");

  uint8_t who_am_i = 0;
  ESP_RETURN_ON_ERROR(
      read_registers(
          imu,
          REG_WHO_AM_I,
          &who_am_i,
          1),
      "aurora_imu",
      "WHO_AM_I read");

  if (who_am_i != 0x68 &&
      who_am_i != 0x69) {
    return ESP_ERR_INVALID_RESPONSE;
  }

  // Wake device, 100 Hz output, DLPF enabled,
  // +/-500 deg/s gyro, +/-4g accelerometer.
  ESP_RETURN_ON_ERROR(
      write_register(imu, REG_PWR_MGMT_1, 0x00),
      "aurora_imu",
      "wake");
  ESP_RETURN_ON_ERROR(
      write_register(imu, REG_SMPLRT_DIV, 0x09),
      "aurora_imu",
      "sample divider");
  ESP_RETURN_ON_ERROR(
      write_register(imu, REG_CONFIG, 0x03),
      "aurora_imu",
      "DLPF");
  ESP_RETURN_ON_ERROR(
      write_register(imu, REG_GYRO_CONFIG, 0x08),
      "aurora_imu",
      "gyro range");
  ESP_RETURN_ON_ERROR(
      write_register(imu, REG_ACCEL_CONFIG, 0x08),
      "aurora_imu",
      "accel range");

  return ESP_OK;
}

esp_err_t aurora_imu_read(
    aurora_imu_t* imu,
    aurora_imu_sample_t* sample) {
  uint8_t raw[14];

  ESP_RETURN_ON_ERROR(
      read_registers(
          imu,
          REG_ACCEL_XOUT_H,
          raw,
          sizeof(raw)),
      "aurora_imu",
      "sensor read");

  const float g = 9.80665f;
  const float accel_scale = g / 8192.0f;
  const float gyro_scale =
      0.017453292519943295f / 65.5f;

  sample->accel_mps2[0] =
      (float)be_i16(&raw[0]) * accel_scale;
  sample->accel_mps2[1] =
      (float)be_i16(&raw[2]) * accel_scale;
  sample->accel_mps2[2] =
      (float)be_i16(&raw[4]) * accel_scale;

  sample->gyro_rps[0] =
      (float)be_i16(&raw[8]) * gyro_scale;
  sample->gyro_rps[1] =
      (float)be_i16(&raw[10]) * gyro_scale;
  sample->gyro_rps[2] =
      (float)be_i16(&raw[12]) * gyro_scale;

  return ESP_OK;
}

void aurora_imu_deinit(
    aurora_imu_t* imu) {
  if (imu->device != NULL) {
    i2c_master_bus_rm_device(imu->device);
    imu->device = NULL;
  }
}
