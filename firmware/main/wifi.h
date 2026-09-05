#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include <stdbool.h>

esp_err_t aurora_wifi_start(void);

bool aurora_wifi_wait_connected(
    TickType_t timeout_ticks);
