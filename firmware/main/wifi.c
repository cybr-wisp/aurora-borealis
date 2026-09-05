#include "sdkconfig.h"
#include "wifi.h"

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

#include <string.h>

static const char* TAG = "aurora_wifi";
static EventGroupHandle_t s_events;
static const EventBits_t CONNECTED_BIT = BIT0;

static void event_handler(
    void* arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void* event_data) {
  (void)arg;
  (void)event_data;

  if (event_base == WIFI_EVENT &&
      event_id == WIFI_EVENT_STA_START) {
    esp_wifi_connect();
    return;
  }

  if (event_base == WIFI_EVENT &&
      event_id == WIFI_EVENT_STA_DISCONNECTED) {
    xEventGroupClearBits(s_events, CONNECTED_BIT);
    esp_wifi_connect();
    return;
  }

  if (event_base == IP_EVENT &&
      event_id == IP_EVENT_STA_GOT_IP) {
    xEventGroupSetBits(s_events, CONNECTED_BIT);
  }
}

esp_err_t aurora_wifi_start(void) {
  s_events = xEventGroupCreate();
  if (s_events == NULL) {
    return ESP_ERR_NO_MEM;
  }

  ESP_RETURN_ON_ERROR(
      esp_netif_init(),
      TAG,
      "esp_netif_init");

  ESP_RETURN_ON_ERROR(
      esp_event_loop_create_default(),
      TAG,
      "event loop");

  esp_netif_create_default_wifi_sta();

  wifi_init_config_t init_config =
      WIFI_INIT_CONFIG_DEFAULT();

  ESP_RETURN_ON_ERROR(
      esp_wifi_init(&init_config),
      TAG,
      "esp_wifi_init");

  ESP_RETURN_ON_ERROR(
      esp_event_handler_register(
          WIFI_EVENT,
          ESP_EVENT_ANY_ID,
          event_handler,
          NULL),
      TAG,
      "Wi-Fi handler");

  ESP_RETURN_ON_ERROR(
      esp_event_handler_register(
          IP_EVENT,
          IP_EVENT_STA_GOT_IP,
          event_handler,
          NULL),
      TAG,
      "IP handler");

  wifi_config_t wifi_config = {0};

  strlcpy(
      (char*)wifi_config.sta.ssid,
      CONFIG_AURORA_WIFI_SSID,
      sizeof(wifi_config.sta.ssid));

  strlcpy(
      (char*)wifi_config.sta.password,
      CONFIG_AURORA_WIFI_PASSWORD,
      sizeof(wifi_config.sta.password));

  wifi_config.sta.threshold.authmode =
      WIFI_AUTH_WPA2_PSK;

  ESP_RETURN_ON_ERROR(
      esp_wifi_set_mode(WIFI_MODE_STA),
      TAG,
      "station mode");

  ESP_RETURN_ON_ERROR(
      esp_wifi_set_config(
          WIFI_IF_STA,
          &wifi_config),
      TAG,
      "station config");

  ESP_RETURN_ON_ERROR(
      esp_wifi_start(),
      TAG,
      "esp_wifi_start");

  ESP_LOGI(TAG, "station started");
  return ESP_OK;
}

bool aurora_wifi_wait_connected(
    TickType_t timeout_ticks) {
  const EventBits_t bits =
      xEventGroupWaitBits(
          s_events,
          CONNECTED_BIT,
          pdFALSE,
          pdTRUE,
          timeout_ticks);

  return (bits & CONNECTED_BIT) != 0;
}

