#include "sdkconfig.h"
#include "sensor_task.h"
#include "telemetry_task.h"
#include "time_sync.h"
#include "wifi.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

static const char* TAG = "aurora_main";
static aurora_sensor_ring_t s_ring;

static esp_err_t create_i2c_bus(
    i2c_master_bus_handle_t* bus) {
  const i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = CONFIG_AURORA_I2C_SDA,
      .scl_io_num = CONFIG_AURORA_I2C_SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  return i2c_new_master_bus(
      &bus_config,
      bus);
}

void app_main(void) {
  esp_err_t nvs_result = nvs_flash_init();

  if (nvs_result == ESP_ERR_NVS_NO_FREE_PAGES ||
      nvs_result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    nvs_result = nvs_flash_init();
  }

  ESP_ERROR_CHECK(nvs_result);

  aurora_sensor_ring_init(&s_ring);

  ESP_ERROR_CHECK(aurora_wifi_start());

  if (!aurora_wifi_wait_connected(
          pdMS_TO_TICKS(30000))) {
    ESP_LOGW(
        TAG,
        "Wi-Fi not connected yet; tasks will keep retrying");
  } else {
    aurora_time_sync();
  }

  i2c_master_bus_handle_t i2c_bus = NULL;

  if (!CONFIG_AURORA_FAKE_SENSORS) {
    ESP_ERROR_CHECK(create_i2c_bus(&i2c_bus));
  }

  const aurora_sensor_task_config_t sensor_config = {
      .ring = &s_ring,
      .i2c_bus = i2c_bus,
      .fake_sensors = CONFIG_AURORA_FAKE_SENSORS,
  };

  const aurora_telemetry_task_config_t telemetry_config = {
      .ring = &s_ring,
      .sensor_id = CONFIG_AURORA_SENSOR_ID,
      .engine_ipv4 = CONFIG_AURORA_ENGINE_IPV4,
      .engine_port = CONFIG_AURORA_ENGINE_PORT,
  };

  ESP_ERROR_CHECK(
      aurora_sensor_task_start(
          &sensor_config));

  ESP_ERROR_CHECK(
      aurora_telemetry_task_start(
          &telemetry_config));

  ESP_LOGI(
      TAG,
      "Aurora node started sensor_id=%d fake_sensors=%d",
      CONFIG_AURORA_SENSOR_ID,
      CONFIG_AURORA_FAKE_SENSORS);
}

