# Day 1–4 engineering design record

## 1. Determinism is an evaluation requirement

Each simulated sensor owns an RNG derived from `(scenario_seed, sensor_id)`. This avoids hidden coupling where adding one sensor changes every other sensor's random stream. Replaying the same seed must produce the identical observation sequence, including missed detections, packet loss, noise, and clutter.

Why: RMSE/NEES comparisons are meaningless if two filters are evaluated on different stochastic realizations.

## 2. Radar measurements stay native until the estimator

The wire format carries range, azimuth, elevation and their sigmas instead of eagerly converting everything to Cartesian. The EKF/UKF therefore sees the actual nonlinear observation model and the correct diagonal measurement covariance in sensor coordinates.

Why: converting noisy spherical data to Cartesian creates correlated, range-dependent Cartesian noise; pretending that converted XYZ noise is independent would make the covariance model wrong.

## 3. Protobuf over UDP

UDP is deliberately used because sensor telemetry is freshness-sensitive and occasional loss is preferable to head-of-line blocking. Protobuf gives a compact, versioned, language-neutral schema. Field 10 (`sent_monotonic_ns`) is an optional benchmark timestamp; adding it is backward-compatible.

## 4. Bounded SPSC queue before MPMC complexity

Day 2 has one UDP receiver thread and one ingestion/tracking consumer, so the queue is a fixed-capacity SPSC ring. It has no per-message allocation and exposes overflow rather than blocking the receive loop.

Current overflow policy: **drop the arriving message and increment `queue_full`**. The original project plan proposed drop-oldest because fresh data is more valuable; doing safe overwrite semantics in a lock-free queue requires additional per-slot sequencing. Rather than ship a subtly racy overwrite queue, Day 1–4 keeps the simpler correct SPSC implementation and records drop-oldest as a later queue-design improvement.

## 5. ENU is the internal tracking frame

Sensor configuration can be geodetic WGS84. Startup converts WGS84 → ECEF and creates an ENU tangent frame about the scenario reference. All filter states are `[x,y,z,vx,vy,vz]` in ENU.

Why: local tracking math stays numerically well-scaled while sensor deployment coordinates remain physically meaningful.

## 6. EKF covariance uses Joseph form

The EKF update uses

`P = (I-KH) P (I-KH)^T + K R K^T`

rather than only `(I-KH)P`.

Why: Joseph form is more robust to floating-point asymmetry and loss of positive semi-definiteness, which matters when NEES depends on the covariance being trustworthy.

## 7. UKF is a comparison implementation, not automatically the default

EKF and UKF share the same state/process/measurement semantics. UKF uses 13 sigma points for the six-dimensional state and circular means for azimuth/elevation.

Current coordinated-turn result: EKF **3.434 m** versus UKF **3.434 m** RMSE. The difference is negligible, so there is no evidence yet to pay the UKF's extra compute cost by default.

## 8. Accuracy and consistency are separate acceptance criteria

RMSE measures tracking error. NEES measures whether the covariance is statistically honest:

`NEES = (x_true - x_est)^T P^-1 (x_true - x_est)`

For six state dimensions, the per-step 95% chi-square interval is **[1.237, 14.449]**.

This separation already exposed a real issue. Under 30% detection dropout + 10% packet loss, the abrupt-maneuver adaptive EKF reaches **4.137 m mean RMSE** across 20 seeds, but only **77.84%** of NEES samples are in the 95% interval on average. That means position accuracy is strong while uncertainty calibration still needs work. The next tuning step should target covariance consistency, not simply lower RMSE.

## 9. Maneuver response is innovation-driven

The detector monitors normalized innovation squared (NIS). A persistent high-NIS window temporarily inflates process noise Q, allowing the constant-velocity model to admit larger accelerations. In the seeded abrupt-maneuver run, the current recovery-to-<8m window improves from **0.8s** to **0.1s**.

## 10. Metrics are reproducible artifacts

`experiments/day1_4_metrics.py` writes the exact JSON committed in `experiments/results/day1_4_metrics.json`. The degraded-sensing claim uses 20 deterministic seeds rather than a single selected trial.
