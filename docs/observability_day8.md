# Day 8 — Operator visualization and observability

The operator console is a Next.js + D3 dashboard connected to a live WebSocket
state stream.

## State contract

The dashboard receives:

- active tracks and lifecycle state
- ENU position and velocity
- 2D position covariance for uncertainty ellipses
- NEES history
- per-sensor configured and adaptive measurement-noise ratios
- sensor health and packet-loss counters
- observations per second, active-track count and p99 latency

The browser does not implement estimation logic. It only renders state and sends
operator commands.

## Software live feed

`simulation/live_operator.py` is the software-only live feed used before
physical HIL is available. It executes real EKF/UKF updates from Aurora's
existing benchmark implementation and exposes deterministic failure controls.

Supported commands:

- kill / restore sensor
- inject / remove range bias
- add / remove measurement noise
- trigger maneuver
- add target
- toggle EKF / UKF
- reset

This bridge is deliberately isolated behind a WebSocket state contract so a
future C++ engine publisher or hardware-backed feed can replace it without
changing dashboard components.

## Run

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
