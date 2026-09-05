#include "time_sync.h"

#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <sys/time.h>
#include <time.h>

static const char* TAG = "aurora_time";

bool aurora_time_sync(void) {
  esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
  esp_sntp_setservername(0, "pool.ntp.org");
  esp_sntp_init();

  for (int attempt = 0; attempt < 30; ++attempt) {
    time_t now;
    time(&now);

    if (now > 1700000000) {
      ESP_LOGI(TAG, "SNTP time synchronized");
      return true;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }

  ESP_LOGW(TAG, "SNTP sync timeout");
  return false;
}

double aurora_epoch_sec(void) {
  struct timeval now;
  gettimeofday(&now, NULL);

  return (double)now.tv_sec +
         (double)now.tv_usec / 1000000.0;
}
