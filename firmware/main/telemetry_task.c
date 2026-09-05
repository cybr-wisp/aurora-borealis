#include "telemetry_task.h"

#include "esp_log.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "proto_encoder.h"
#include "wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>

static const char* TAG = "aurora_telemetry";

typedef struct {
  aurora_telemetry_task_config_t config;
} task_context_t;

static int open_udp_socket(
    const aurora_telemetry_task_config_t* config,
    struct sockaddr_in* destination) {
  const int socket_fd =
      socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (socket_fd < 0) {
    return -1;
  }

  memset(destination, 0, sizeof(*destination));
  destination->sin_family = AF_INET;
  destination->sin_port =
      htons(config->engine_port);

  if (inet_pton(
          AF_INET,
          config->engine_ipv4,
          &destination->sin_addr) != 1) {
    close(socket_fd);
    return -1;
  }

  return socket_fd;
}

static void telemetry_task(void* raw_context) {
  task_context_t* context =
      (task_context_t*)raw_context;

  int socket_fd = -1;
  struct sockaddr_in destination = {0};

  uint64_t sent = 0;
  uint64_t send_failures = 0;

  for (;;) {
    if (!aurora_wifi_wait_connected(
            pdMS_TO_TICKS(1000))) {
      continue;
    }

    if (socket_fd < 0) {
      socket_fd =
          open_udp_socket(
              &context->config,
              &destination);

      if (socket_fd < 0) {
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
    }

    aurora_sensor_sample_t sample;

    if (!aurora_sensor_ring_pop(
            context->config.ring,
            &sample)) {
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    aurora_observation_wire_t observation = {
        .sensor_id = context->config.sensor_id,
        .sequence_number = sample.sequence_number,
        .timestamp_sec = sample.timestamp_sec,
        .range_m = sample.distance_m,
        .azimuth_rad = 0.0,
        .elevation_rad = 0.0,
        .range_sigma = 0.03,
        .azimuth_sigma = 0.02,
        .elevation_sigma = 0.02,
        // ESP32 and host steady clocks do not share an epoch.
        // Keep this zero outside loopback benchmarks; timestamp_sec is
        // the cross-node NTP-synchronized timestamp.
        .sent_monotonic_ns = 0,
    };

    uint8_t payload[128];

    const size_t payload_size =
        aurora_encode_observation(
            &observation,
            payload,
            sizeof(payload));

    if (payload_size == 0) {
      ESP_LOGE(TAG, "Protobuf output buffer too small");
      continue;
    }

    const int result =
        sendto(
            socket_fd,
            payload,
            payload_size,
            0,
            (const struct sockaddr*)&destination,
            sizeof(destination));

    if (result < 0) {
      ++send_failures;
      close(socket_fd);
      socket_fd = -1;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    ++sent;

    if (sent % 500U == 0U) {
      ESP_LOGI(
          TAG,
          "sent=%llu send_failures=%llu ring_depth=%u",
          (unsigned long long)sent,
          (unsigned long long)send_failures,
          (unsigned)aurora_sensor_ring_size(
              context->config.ring));
    }
  }
}

esp_err_t aurora_telemetry_task_start(
    const aurora_telemetry_task_config_t* config) {
  task_context_t* context =
      calloc(1, sizeof(*context));

  if (context == NULL) {
    return ESP_ERR_NO_MEM;
  }

  context->config = *config;

  const BaseType_t result =
      xTaskCreatePinnedToCore(
          telemetry_task,
          "TelemetryTask",
          4096,
          context,
          7,
          NULL,
          0);

  if (result != pdPASS) {
    free(context);
    return ESP_FAIL;
  }

  return ESP_OK;
}
