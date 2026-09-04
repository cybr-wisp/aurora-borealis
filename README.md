# Aurora Borealis

Real-time multi-sensor tracking research system built around deterministic simulation, UDP/Protobuf ingestion, WGS84→ECEF→ENU geometry, EKF/UKF estimation, NEES consistency testing, and innovation-based maneuver detection.

## Day 1–4 status

| Capability | Status | Measured result |
|---|---|---|
| Deterministic imperfect-radar simulation | complete | 10,000-sample validation; observed packet loss 9.99% for configured 10% |
| C++ UDP + Protobuf ingestion | complete | bounded SPSC queue, validation, parse+enqueue benchmark in CI |
| WGS84/ECEF/ENU geometry | complete | round-trip unit tests target <1 mm |
| EKF | complete | 3-radar constant-velocity RMSE 1.37 m |
| UKF comparison | complete | turn RMSE EKF 3.43 m vs UKF 3.43 m |
| NEES consistency | complete | 6-DoF 95% bounds [1.237, 14.449] |
| Maneuver detector / adaptive Q | complete | recovery 0.8s → 0.1s in seeded abrupt-maneuver run |

### Degraded-sensing Monte Carlo

20 deterministic seeds, three radars, **70% detection probability (30% dropout) + 10% packet loss per radar**:

- Coordinated turn: **4.45 m mean RMSE**, **4.76 m p95**, 100% of runs below 5 m, **94.56%** mean NEES-in-95%-bounds.
- Abrupt acceleration maneuver + adaptive Q: **4.14 m mean RMSE**, **4.44 m p95**, 100% of runs below 5 m, but only **77.84%** mean NEES-in-bounds. Accuracy clears the current target; uncertainty calibration does not yet.

The committed results are generated, not hand-entered: `experiments/results/tracking_benchmark.json` records seeds and conditions.

## Build and test

```bash
python3 -m pip install -r requirements.txt
python3 -m pytest -q simulation/tests
python3 experiments/tracking_benchmark.py
sudo apt-get update
sudo apt-get install -y cmake ninja-build g++ libeigen3-dev libprotobuf-dev protobuf-compiler libgtest-dev
cmake -S engine -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
./build/bench_ingestion
```

Generate the Python Protobuf binding before sending simulated observations over UDP:

```bash
protoc -I engine/proto --python_out=simulation/sim/network engine/proto/observation.proto
```

## Design notes

See [`docs/tracking_design.md`](docs/tracking_design.md) for the decisions and tradeoffs behind deterministic RNG ownership, SPSC backpressure, Protobuf, ENU tracking, Joseph-form covariance updates, EKF/UKF parity, and NEES/NIS validation.

