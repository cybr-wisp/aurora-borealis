idf_component_register(
    SRCS
        "main.c"
        "ring_buffer.c"
        "proto_encoder.c"
        "imu_driver.c"
        "tof_driver.c"
        "calibration.c"
        "wifi.c"
        "time_sync.c"
        "sensor_task.c"
        "telemetry_task.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_i2c esp_driver_gpio
        driver
        esp_event
        esp_netif
        esp_timer
        esp_wifi
        lwip
        nvs_flash
)


