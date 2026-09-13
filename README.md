<div align="center">

# Aurora Borealis

**Multi-sensor tracking and state-estimation under unreliable sensing.**

[![Tests](https://github.com/cybr-wisp/aurora-borealis/actions/workflows/test.yaml/badge.svg)](https://github.com/cybr-wisp/aurora-borealis/actions/workflows/test.yaml)
![C++20](https://img.shields.io/badge/C++-20-00599C?logo=cplusplus&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.11%2B-3776AB?logo=python&logoColor=white)
![ESP-IDF](https://img.shields.io/badge/ESP32-ESP--IDF-E7352C?logo=espressif&logoColor=white)
![License](https://img.shields.io/badge/License-Apache%202.0-28A745)

</div>

Aurora Borealis is a research-oriented tracking system for studying how
multi-radar estimators behave under missed detections, packet loss, sensor
noise, and target maneuvers.

The project spans deterministic radar simulation, nonlinear Bayesian
filtering, interacting multiple-model estimation, statistical consistency
testing, a native C++ ingestion / association pipeline, a live operator
console, and an ESP32 sensor-node firmware path toward physical
hardware-in-the-loop testing.

## Table of contents

- [1.0 Results](#10-results)
- [2.0 System](#20-system)
- [3.0 Why this is non-trivial](#30-why-this-is-non-trivial)
- [4.0 Baselines](#40-baselines)
- [5.0 Native performance](#50-native-performance)
- [6.0 Robustness experiments](#60-robustness-experiments)
- [7.0 Engineering decisions](#70-engineering-decisions)
- [8.0 Reproduce it](#80-reproduce-it)
- [9.0 Repository layout](#90-repository-layout)
- [10.0 Scope and limitations](#100-scope-and-limitations)
- [11.0 Evidence](#110-evidence)

---

## 1.0 Results

The final estimator was frozen before evaluation on **50 previously unseen
Monte Carlo seeds per trajectory regime**.

Every run used:

- 3 independent radars
- 30% missed detections per radar
- 10% packet loss per radar
- noisy range / azimuth / elevation measurements

| Held-out metric | Coordinated turn | Abrupt maneuver |
|---|---:|---:|
| Runs | 50 | 50 |
| Mean RMSE | **4.411 m** | **4.490 m** |
| p95 RMSE | **4.658 m** | **4.839 m** |
| Maximum RMSE | 5.045 m | 5.184 m |
| Runs below 5 m | **98%** | **98%** |
| Empirical 6D NEES coverage | **93.76%** | **91.25%** |
| Mean 6D NEES | 5.233 | **5.946** |
| Mean 3D NIS | **2.946** | **2.959** |

**98 of 100 held-out scenario-runs remained below 5 m RMSE.**

The nominal expectations are `E[NEES] = 6` for the six-state consistency
test and `E[NIS] = 3` for the radar innovation. The abrupt-maneuver mean
NEES of **5.946** and mean NIS of **2.959** are close to their nominal
expectations, although empirical NEES interval coverage under maneuver
remains below the nominal 95% target.

The repository preserves that failure rather than tuning against the final
seeds.

---

## 2.0 System

Aurora has three connected paths: research estimation, native ingestion, and
live operator visualization, with an embedded sensor node feeding the same
pipeline as the software simulator.

![Architecture diagram](docs/architecture-diagram.png)

### Research estimation path

Two constant-acceleration EKFs, state `[x, y, z, vx, vy, vz, ax, ay, az]`,
combined in an interacting multiple-model (IMM) estimator: smooth-model
jerk Q = 0.1, maneuver-model jerk Q = 100.0, transition probabilities
0.01 / 0.10. Mode probabilities update from radar measurement
likelihoods; IMM mixing carries both within-model covariance and
between-model disagreement into the fused posterior.

### Native systems path

`UDP → Protobuf decode → validation → bounded SPSC queue → gating /
association → tracking`. UDP is intentional: sensor telemetry is
freshness-sensitive, so occasional loss is preferable to transport-level
head-of-line blocking.

### Live operator console

`dashboard/` is a Next.js + D3 console driven by a live WebSocket state
stream (tracks, covariance ellipses, NEES history, sensor health,
throughput). The browser only renders state and sends operator commands —
no estimation logic runs client-side. `simulation/live_operator.py`
currently drives it with real EKF/UKF updates and deterministic failure
controls (kill/restore a sensor, inject noise or bias, trigger a maneuver,
toggle EKF/UKF), sitting behind the same state contract a hardware-backed
feed will later use.

### Embedded sensor node

`firmware/` targets an ESP32-S3, built so the real-time pipeline is
proven before physical hardware arrives. `CONFIG_AURORA_FAKE_SENSORS=y`
(current default) exercises the real FreeRTOS/buffering/Protobuf/UDP path
with deterministic fake IMU/ToF values; the real MPU6050 driver is
implemented and untested on hardware, and the real VL53L1X path is not
yet implemented. See [`docs/embedded_day6.md`](docs/embedded_day6.md) for
the full real-vs-fake breakdown.

---

## 3.0 Why this is non-trivial

### Deterministic stochastic simulation

Each radar owns an independent pseudo-random stream derived from

```text
(scenario seed, sensor id)
```

so adding or removing one sensor does not perturb another sensor's noise
sequence.

This makes estimator comparisons reproducible: two filters evaluated on the
same seed see the same missed detections, packet loss, and measurement noise.

### Native nonlinear measurements

Radar observations remain in:

```text
range
azimuth
elevation
```

until the estimator update.

They are not prematurely converted into Cartesian measurements with an
incorrect independent-noise assumption.

### Accuracy != consistency

Aurora evaluates both:

```text
RMSE  -> how wrong is the estimate?
NEES  -> is the reported state covariance statistically credible?
NIS   -> is the measurement innovation statistically credible?
```

For the six-dimensional position / velocity state,

```text
NEES = eᵀ P⁻¹ e
```

with nominal per-step 95% chi-squared bounds:

```text
[1.237, 14.449]
```

This distinction caught cases where tracking error was low while covariance
was still miscalibrated.

### Held-out evaluation

Estimator development used:

```text
20263000–20263019
```

Final validation used a separate untouched set:

```text
20264000–20264049
```

The estimator was frozen before those 50 final seeds were evaluated.

Those seeds are now treated as **spent**: they may be rerun to reproduce the
published result, but they are not used for further model selection.

---

## 4.0 Baselines

The simpler constant-velocity estimator establishes the value of
multi-sensor fusion before maneuver modeling is introduced.

| Configuration | Position RMSE |
|---|---:|
| Raw radar measurements | 17.104 m |
| 1-radar EKF | 2.464 m |
| 3-radar EKF | **1.374 m** |

Three-radar fusion reduced RMSE by **91.97%** relative to the raw
measurement baseline.

For a coordinated-turn baseline, EKF and UKF were effectively tied:

```text
EKF RMSE  3.434 m
UKF RMSE  3.434 m
```

so the project does not assume a UKF is automatically superior simply
because the measurement model is nonlinear.

---

## 5.0 Native performance

Benchmarks below come from a **Linux Release build in GitHub Actions**.

### Ingestion

```text
payload                     69 bytes
messages                    300,000
parse + enqueue throughput  7,891,090 msg/s
p50                         0.090 µs
p95                         0.091 µs
p99                         0.100 µs
```

Headline:

**~7.9M messages/s in-process Protobuf parse + enqueue throughput.**

This is a microbenchmark of the in-process ingestion path, **not**
end-to-end UDP network throughput.

### Data association

At 100 tracks:

```text
p50  37.550 µs
p95  38.432 µs
p99  38.573 µs
```

Headline:

**~39 µs p99 association latency at 100 tracks.**

![Association scaling](docs/figures/association_scaling.png)

---

## 6.0 Robustness experiments

Aurora also evaluates estimator behavior as sensing quality changes.

<table>
  <tr>
    <td width="50%">
      <img src="docs/figures/packet_loss.png" width="100%" alt="Packet-loss sweep" />
      <p align="center"><sub><b>Packet loss</b></sub></p>
    </td>
    <td width="50%">
      <img src="docs/figures/sensor_count.png" width="100%" alt="Sensor-count sweep" />
      <p align="center"><sub><b>Sensor count</b></sub></p>
    </td>
  </tr>
  <tr>
    <td width="50%">
      <img src="docs/figures/sensor_outage.png" width="100%" alt="Sensor-outage experiment" />
      <p align="center"><sub><b>Sensor outage</b></sub></p>
    </td>
    <td width="50%">
      <img src="docs/figures/noise_sweep.png" width="100%" alt="Noise sweep" />
      <p align="center"><sub><b>Measurement noise</b></sub></p>
    </td>
  </tr>
</table>

The raw generated results live under:

```text
experiments/results/
```

rather than being manually copied into documentation.

---

## 7.0 Engineering decisions

| Decision | Reason |
|---|---|
| ENU state frame | Keeps local tracking numerically well-scaled while allowing WGS84 sensor configuration |
| Native spherical radar observations | Preserves the nonlinear observation model and correct sensor-coordinate covariance |
| Joseph covariance update | Better numerical symmetry / PSD behavior for consistency testing |
| Deterministic per-sensor RNG | Reproducible filter comparisons |
| Bounded SPSC queue | No per-message allocation in the one-producer / one-consumer ingestion path |
| IMM over one globally inflated Q | Allows smooth and maneuver hypotheses to coexist |
| Held-out seeds | Separates parameter selection from final evaluation |
| WebSocket state contract for the dashboard | Lets the live feed backend change (software → hardware) without touching frontend components |
| Fake-mode-first firmware | Real-time pipeline, buffering, and wire format proven before physical sensors arrive |

Detailed design discussion is in
[`docs/tracking_design.md`](docs/tracking_design.md).

---

## 8.0 Reproduce it

### Python evaluation

```bash
git clone https://github.com/cybr-wisp/aurora-borealis.git
cd aurora-borealis

python -m venv .venv
```

Windows:

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements-repro.txt
```

Linux / macOS:

```bash
source .venv/bin/activate
python -m pip install -r requirements-repro.txt
```

Run the simulation tests:

```bash
python -m pytest simulation/tests -q
```

Run the general evaluation suite:

```bash
python experiments/run_all.py
```

### Reproduce the published held-out IMM result

The estimator used for the published final validation was frozen at:

```text
57dba08bc234507da7b465f533b00c1e77380ea5
```

The committed result is:

```text
experiments/results/imm_final_heldout_validation.json
```

Re-running the final seeds is appropriate for reproduction, but they should
not be reused for subsequent tuning.

### Native Linux Release build

```bash
sudo apt-get update
sudo apt-get install -y \
  cmake \
  ninja-build \
  g++ \
  libeigen3-dev \
  libprotobuf-dev \
  protobuf-compiler \
  libgtest-dev

cmake -S engine -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build

ctest --test-dir build --output-on-failure
python experiments/system_benchmarks.py
```

CI executes the same Release build and uploads generated results and figures
as evaluation artifacts.

### Live operator console

Terminal 1:

```powershell
.\.venv\Scripts\Activate.ps1
python simulation/live_operator.py
```

Terminal 2:

```powershell
cd dashboard
npm install
npm run typecheck
npm run build
npm run dev
```

Open `http://localhost:3000`.

### Embedded firmware (fake-sensor mode)

Requires ESP-IDF installed and targeting the ESP32-S3:

```powershell
cd firmware
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

Set Wi-Fi credentials, the Aurora engine's IPv4, UDP port, and sensor ID
under the `Aurora Borealis sensor node` menu. For fake-mode network
integration, run the desktop engine on UDP 46000 and point the ESP32
node's engine IPv4 at the host machine's LAN address.

---

## 9.0 Repository layout

```text
aurora-borealis/
├── engine/                 C++ ingestion, geometry, estimation, association
├── simulation/             deterministic radar + trajectory simulation, live operator bridge
├── dashboard/              Next.js + D3 live operator console (WebSocket state stream)
├── firmware/               ESP32-S3 sensor-node firmware (fake-mode default, real MPU6050 path)
├── experiments/            evaluation and benchmark drivers
│   └── results/            committed machine-readable results
├── docs/
│   ├── figures/            generated evaluation figures
│   ├── math_design.md
│   ├── tracking_design.md
│   ├── observability_day8.md
│   └── embedded_day6.md
└── .github/workflows/      test + Linux evaluation CI
```

---

## 10.0 Scope and limitations

Aurora's strongest claims are based on simulation and Linux CI benchmarks.

The final held-out experiment achieved:

```text
98% of scenario-runs below 5 m RMSE
```

not 100%.

Abrupt-maneuver empirical NEES coverage was:

```text
91.25%
```

not 95%.

The ~7.9M msg/s result is an in-process parsing / enqueue benchmark, not
end-to-end network throughput.

The dashboard and live operator console are validated against the software
estimator only; no C++-engine-backed live feed exists yet.

The firmware builds and runs its full real-time pipeline against
deterministic fake sensor values (`CONFIG_AURORA_FAKE_SENSORS=y`), and the
real IMU path is implemented but untested against physical hardware. The
real ToF path is not implemented. Physical hardware-in-the-loop validation
is not claimed until a hardware experiment is completed and recorded.

---

## 11.0 Evidence

Key machine-readable artifacts:

```text
experiments/results/imm_final_heldout_validation.json
experiments/results/tracking_benchmark.json
experiments/results/ingestion_benchmark.csv
experiments/results/association_benchmark.csv
experiments/results/backpressure_benchmark.csv
```

Reproducibility environment:

```text
requirements-repro.txt
docs/repro_environment.txt
```

Every headline number in this README should be traceable to one of these
artifacts.