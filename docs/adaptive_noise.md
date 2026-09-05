# Adaptive measurement-noise estimation

Aurora maintains an independent online measurement-noise estimate for each
`sensor_id`.

For each range/azimuth/elevation channel, the estimator compares measured
innovation energy with the variance already explained by the predicted track
covariance:

`R ~= mean(v^2) - mean(H P_pred H^T)`

The result is windowed, clamped relative to configured sensor noise, and
exponentially smoothed. Aurora currently adapts diagonal measurement variances
because the radar sensor model assumes independent range, azimuth and elevation
noise.

Tracks are predicted once per timestamp. Measurements are grouped by sensor and
then sequentially fused, allowing multiple sensors to update one track at the
same timestamp without applying the process model multiple times.

The deterministic failure injector supports complete outages, packet-loss
increases, additive bias and additional Gaussian measurement noise.

Physical ESP32 hardware-in-the-loop validation remains pending hardware.
