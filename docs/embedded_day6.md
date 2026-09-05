# Day 6 â€” Embedded firmware before hardware

The firmware is structured so the networking and real-time pipeline can be
built before the physical ESP32/IMU/ToF parts arrive.

## Runtime

`SensorTask` runs at 100 Hz and writes timestamped samples into a bounded
ring buffer. It measures actual sample interval and maximum absolute scheduling
jitter.

`TelemetryTask` runs independently. It waits for Wi-Fi connectivity, consumes
samples, encodes the same `Observation` Protobuf wire schema used by the C++
engine, and transmits UDP to Aurora. `timestamp_sec` is wall-clock time after
SNTP synchronization. The ESP32 does not populate `sent_monotonic_ns` because
its steady clock is not comparable to the host steady clock.

The firmware ring buffer uses **drop-oldest** semantics. If Wi-Fi is down,
TelemetryTask stops draining while SensorTask continues sampling; once the
buffer fills, the oldest samples are replaced so the node preserves recent
state.

## Fake backend

`CONFIG_AURORA_FAKE_SENSORS=y` is the default. It produces deterministic IMU
and ToF values while exercising the real FreeRTOS, buffering, Protobuf and UDP
code paths.

This lets Day 6 be tested without claiming physical hardware validation.

## Real MPU6050 path

The MPU6050 adapter uses the ESP-IDF I2C master driver directly:

- 400 kHz I2C
- WHO_AM_I validation
- wake from sleep
- 100 Hz divider
- DLPF configuration
- +/-500 deg/s gyro
- +/-4g accelerometer
- retry/backoff around I2C transactions
- stationary startup bias calibration

## ToF boundary

The VL53L1X real backend is deliberately **not guessed** without hardware.
The adapter returns `ESP_ERR_NOT_SUPPORTED` until ST's Ultra Lite Driver is
added and tested against the actual module. The deterministic fake backend is
used for Day 6.

That means:

- Day 6 firmware architecture: complete once fake-mode ESP-IDF build passes
- Day 6 hardware validation: pending the board/sensors

## Build when ESP-IDF is installed

```powershell
cd firmware
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

Set the Wi-Fi credentials, Aurora engine IPv4, UDP port and sensor ID under the
`Aurora Borealis sensor node` menu.

For fake-mode network integration, run the desktop engine on UDP 46000 and set
the ESP32 node's engine IPv4 to the host machine's LAN address.

