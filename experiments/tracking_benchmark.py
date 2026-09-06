from __future__ import annotations

import json
import math
import platform
import time
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import numpy as np
from simulation.sim.sensors.radar import RadarConfig, RadarSensor, spherical_to_cartesian
from simulation.sim.trajectory import constant_velocity, coordinated_turn, acceleration_maneuver

SEED = 20260903
DT = 0.1
NEES_BOUNDS = (1.237344245791203, 14.44937533544792)


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def h(x: np.ndarray, sensor: np.ndarray) -> np.ndarray:
    d = x[:3] - sensor
    r = np.linalg.norm(d)
    rho = math.hypot(d[0], d[1])
    return np.array([r, math.atan2(d[1], d[0]), math.atan2(d[2], rho)], dtype=float)


def H(x: np.ndarray, sensor: np.ndarray) -> np.ndarray:
    dx, dy, dz = x[:3] - sensor
    rho2 = dx * dx + dy * dy
    rho = math.sqrt(rho2)
    r2 = rho2 + dz * dz
    r = math.sqrt(r2)
    out = np.zeros((3, 6))
    out[0, :3] = [dx / r, dy / r, dz / r]
    out[1, :3] = [-dy / rho2, dx / rho2, 0.0]
    out[2, :3] = [-dx * dz / (r2 * rho), -dy * dz / (r2 * rho), rho / r2]
    return out


def residual(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    y = a - b
    y[1] = wrap(float(y[1]))
    y[2] = wrap(float(y[2]))
    return y


def transition(dt: float) -> np.ndarray:
    f = np.eye(6)
    f[:3, 3:] = np.eye(3) * dt
    return f


def process_noise(dt: float, q: float) -> np.ndarray:
    out = np.zeros((6, 6))
    block = np.array([[dt**3 / 3, dt**2 / 2], [dt**2 / 2, dt]]) * q
    for i in range(3):
        out[np.ix_([i, i + 3], [i, i + 3])] = block
    return out


class EKF:
    def __init__(self, x: np.ndarray, p: np.ndarray, q: float):
        self.x, self.p, self.q = x.copy(), p.copy(), q

    def predict(self, dt: float, scale: float = 1.0):
        f = transition(dt)
        self.x = f @ self.x
        self.p = f @ self.p @ f.T + process_noise(dt, self.q * scale)
        self.p = (self.p + self.p.T) / 2

    def update(self, z: np.ndarray, r_cov: np.ndarray, sensor: np.ndarray) -> float:
        hh = H(self.x, sensor)
        y = residual(z, h(self.x, sensor))
        s = hh @ self.p @ hh.T + r_cov
        k = np.linalg.solve(s, hh @ self.p).T
        self.x = self.x + k @ y
        i = np.eye(6)
        ikh = i - k @ hh
        self.p = ikh @ self.p @ ikh.T + k @ r_cov @ k.T
        self.p = (self.p + self.p.T) / 2
        return float(y @ np.linalg.solve(s, y))


class UKF:
    def __init__(self, x: np.ndarray, p: np.ndarray, q: float, alpha: float = 0.35, beta: float = 2.0, kappa: float = 0.0):
        self.x, self.p, self.q = x.copy(), p.copy(), q
        n = 6
        self.lam = alpha * alpha * (n + kappa) - n
        self.wm = np.full(2 * n + 1, 1 / (2 * (n + self.lam)))
        self.wc = self.wm.copy()
        self.wm[0] = self.lam / (n + self.lam)
        self.wc[0] = self.wm[0] + (1 - alpha * alpha + beta)

    def sigma(self) -> np.ndarray:
        l = np.linalg.cholesky((6 + self.lam) * self.p)
        return np.column_stack([self.x, *[self.x + l[:, i] for i in range(6)], *[self.x - l[:, i] for i in range(6)]])

    def predict(self, dt: float, scale: float = 1.0):
        f = transition(dt)
        sig = f @ self.sigma()
        self.x = sig @ self.wm
        self.p = process_noise(dt, self.q * scale)
        for i in range(sig.shape[1]):
            d = sig[:, i] - self.x
            self.p += self.wc[i] * np.outer(d, d)
        self.p = (self.p + self.p.T) / 2

    def update(self, z: np.ndarray, r_cov: np.ndarray, sensor: np.ndarray) -> float:
        sig = self.sigma()
        zs = np.column_stack([h(sig[:, i], sensor) for i in range(sig.shape[1])])
        zm = np.zeros(3)
        zm[0] = zs[0] @ self.wm
        for j in (1, 2):
            zm[j] = math.atan2(float(np.sin(zs[j]) @ self.wm), float(np.cos(zs[j]) @ self.wm))
        s = r_cov.copy()
        pxz = np.zeros((6, 3))
        for i in range(sig.shape[1]):
            dx = sig[:, i] - self.x
            dz = residual(zs[:, i], zm)
            s += self.wc[i] * np.outer(dz, dz)
            pxz += self.wc[i] * np.outer(dx, dz)
        k = np.linalg.solve(s, pxz.T).T
        inn = residual(z, zm)
        self.x = self.x + k @ inn
        self.p = self.p - k @ s @ k.T
        self.p = (self.p + self.p.T) / 2
        return float(inn @ np.linalg.solve(s, inn))


def obs_cov(cfg: RadarConfig) -> np.ndarray:
    return np.diag([cfg.range_sigma_m**2, cfg.azimuth_sigma_rad**2, cfg.elevation_sigma_rad**2])


def first_measurement_state(obs, sensor_pos: np.ndarray) -> np.ndarray:
    p = sensor_pos + spherical_to_cartesian(obs.range_m, obs.azimuth_rad, obs.elevation_rad)
    return np.r_[p, [0.0, 0.0, 0.0]]


def build_observations(truth, configs, seed=SEED):
    radars = [RadarSensor(c, seed) for c in configs]
    by_time: dict[float, list[tuple[object, RadarConfig]]] = {}
    for state in truth:
        for radar, cfg in zip(radars, configs):
            for o in radar.observe(state):
                if not o.is_clutter:
                    by_time.setdefault(state.timestamp_sec, []).append((o, cfg))
    return by_time, radars


def run_filter(
    filter_cls,
    truth,
    configs,
    q=3.0,
    adaptive=False,
    seed=SEED,
    nis_trigger=7.815,
    q_scale_max=5.0,
    q_scale_alpha=0.35,
    q_scale_decay=0.20,
):
    observations, _ = build_observations(truth, configs, seed=seed)
    first_t = min(observations)
    o0, c0 = observations[first_t][0]
    x0 = first_measurement_state(o0, np.asarray(c0.position_enu_m))
    p0 = np.diag([400, 400, 400, 2500, 2500, 2500]).astype(float)
    filt = filter_cls(x0, p0, q)
    sq_pos, raw_sq, nees_vals, nis_vals = [], [], [], []
    history: list[float] = []
    q_scale = 1.0
    per_step_rmse: list[float] = []

    # 3-DOF measurement innovation: expected NIS is approximately 3.
    # Rather than jumping between 1x and 10x process noise, adapt Q
    # continuously to the magnitude of recent innovation inconsistency.
    nis_expected = 3.0

    previous_t = first_t
    for state in truth:
        t = state.timestamp_sec
        if t < first_t or t not in observations:
            continue

        if t > first_t:
            filt.predict(t - previous_t, q_scale)

        previous_t = t

        for obs, cfg in observations[t]:
            z = np.array(
                [obs.range_m, obs.azimuth_rad, obs.elevation_rad]
            )

            nis = filt.update(
                z,
                obs_cov(cfg),
                np.asarray(cfg.position_enu_m),
            )

            nis_vals.append(nis)
            history.append(nis)
            history = history[-5:]

            if adaptive and len(history) == 5:
                mean_nis = float(np.mean(history))

                if mean_nis > nis_trigger:
                    desired_scale = float(
                        np.clip(
                            mean_nis / nis_expected,
                            1.0,
                            q_scale_max,
                        )
                    )

                    q_scale = (
                        (1.0 - q_scale_alpha) * q_scale
                        + q_scale_alpha * desired_scale
                    )
                else:
                    # Smoothly return toward the nominal process model.
                    q_scale += q_scale_decay * (1.0 - q_scale)
        truth_x = state.vector()
        e = truth_x - filt.x
        if t >= 5.0:
            sq_pos.append(float(e[:3] @ e[:3]))
            nees_vals.append(float(e @ np.linalg.solve(filt.p, e)))
            per_step_rmse.append(float(np.linalg.norm(e[:3])))
            # Raw metric uses the first available sensor at this timestamp.
            obs, cfg = observations[t][0]
            raw_p = np.asarray(cfg.position_enu_m) + spherical_to_cartesian(obs.range_m, obs.azimuth_rad, obs.elevation_rad)
            raw_sq.append(float(np.sum((truth_x[:3] - raw_p) ** 2)))

    lo, hi = NEES_BOUNDS
    return {
        "rmse_m": math.sqrt(float(np.mean(sq_pos))),
        "raw_rmse_m": math.sqrt(float(np.mean(raw_sq))),
        "rmse_improvement_pct": 100.0 * (1.0 - math.sqrt(float(np.mean(sq_pos))) / math.sqrt(float(np.mean(raw_sq)))),
        "nees_in_95_pct": 100.0 * float(np.mean([(lo <= v <= hi) for v in nees_vals])),
        "nees_mean": float(np.mean(nees_vals)),
        "nis_mean": float(np.mean(nis_vals)),
        "position_errors_m": per_step_rmse,
    }


def recovery_time(errors: list[float], maneuver_end_sec: float, start_eval_sec: float = 5.0, threshold_m: float = 8.0) -> float | None:
    start_idx = max(0, int(round((maneuver_end_sec - start_eval_sec) / DT)))
    for i in range(start_idx, len(errors) - 9):
        if max(errors[i:i+10]) < threshold_m:
            return round((i - start_idx) * DT, 2)
    return None


def day1_sensor_metrics():
    cfg = RadarConfig(sensor_id=3, position_enu_m=(0, 0, 0), range_sigma_m=15.0, azimuth_sigma_rad=0.004, elevation_sigma_rad=0.003, detection_probability=1.0, packet_loss_probability=0.10)
    radar = RadarSensor(cfg, SEED)
    truth = constant_velocity(target_id=1, position0_m=(1800, 600, 250), velocity_mps=(0, 0, 0), duration_sec=1000.0, dt_sec=0.1)
    rerr, aerr, eerr = [], [], []
    true = np.array([math.sqrt(1800**2 + 600**2 + 250**2), math.atan2(600, 1800), math.atan2(250, math.hypot(1800, 600))])
    for s in truth[:10000]:
        obs = radar.observe(s)
        if obs:
            z = np.array([obs[0].range_m, obs[0].azimuth_rad, obs[0].elevation_rad])
            d = residual(z, true)
            rerr.append(d[0]); aerr.append(d[1]); eerr.append(d[2])
    return {
        "samples_delivered": len(rerr),
        "packet_loss_observed_pct": 100.0 * radar.packet_drops / radar.generated_detections,
        "range_noise_mean_m": float(np.mean(rerr)),
        "range_noise_std_m": float(np.std(rerr, ddof=1)),
        "azimuth_noise_std_rad": float(np.std(aerr, ddof=1)),
        "elevation_noise_std_rad": float(np.std(eerr, ddof=1)),
    }


def summarize_runs(runs: list[dict]) -> dict:
    rmse = np.array([r["rmse_m"] for r in runs], dtype=float)
    coverage = np.array([r["nees_in_95_pct"] for r in runs], dtype=float)
    return {
        "runs": len(runs),
        "rmse_mean_m": float(rmse.mean()),
        "rmse_p95_m": float(np.percentile(rmse, 95)),
        "rmse_max_m": float(rmse.max()),
        "runs_below_5m_pct": 100.0 * float(np.mean(rmse < 5.0)),
        "mean_nees_in_95_pct": float(coverage.mean()),
    }


def main():
    nominal = dict(range_sigma_m=10.0, azimuth_sigma_rad=0.0025, elevation_sigma_rad=0.0025)
    one_cfg = [RadarConfig(sensor_id=1, position_enu_m=(0, 0, 0), **nominal)]
    three_cfg = [
        one_cfg[0],
        RadarConfig(sensor_id=2, position_enu_m=(2500, -800, 50), **nominal),
        RadarConfig(sensor_id=3, position_enu_m=(-1800, 1400, 80), **nominal),
    ]
    degraded_cfg = [
        RadarConfig(sensor_id=1, position_enu_m=(0, 0, 0), detection_probability=0.70, packet_loss_probability=0.10, **nominal),
        RadarConfig(sensor_id=2, position_enu_m=(2500, -800, 50), detection_probability=0.70, packet_loss_probability=0.10, **nominal),
        RadarConfig(sensor_id=3, position_enu_m=(-1800, 1400, 80), detection_probability=0.70, packet_loss_probability=0.10, **nominal),
    ]
    cv = constant_velocity(target_id=1, position0_m=(1200, 350, 300), velocity_mps=(75, 12, 0.5), duration_sec=60, dt_sec=DT)
    turn = coordinated_turn(target_id=1, position0_m=(1200, 350, 300), speed_mps=78, heading0_rad=0.15, turn_rate_radps=0.045, vertical_speed_mps=0.5, duration_sec=60, dt_sec=DT)
    maneuver = acceleration_maneuver(target_id=1, position0_m=(1200, 350, 300), velocity0_mps=(75, 12, 0.5), acceleration_mps2=(-7, 10, 0), maneuver_start_sec=20, maneuver_end_sec=25, duration_sec=60, dt_sec=DT)

    t0 = time.perf_counter()
    one_ekf = run_filter(EKF, cv, one_cfg, q=0.02)
    three_ekf = run_filter(EKF, cv, three_cfg, q=0.02)
    turn_ekf = run_filter(EKF, turn, three_cfg, q=12.0)
    turn_ukf = run_filter(UKF, turn, three_cfg, q=12.0)
    maneuver_static = run_filter(EKF, maneuver, three_cfg, q=2.5, adaptive=False)
    maneuver_adaptive = run_filter(EKF, maneuver, three_cfg, q=2.5, adaptive=True)

    mc_seeds = list(range(20260900, 20260920))
    degraded_turn_runs = [run_filter(EKF, turn, degraded_cfg, q=12.0, seed=seed) for seed in mc_seeds]
    degraded_maneuver_runs = [run_filter(EKF, maneuver, degraded_cfg, q=2.5, adaptive=True, seed=seed) for seed in mc_seeds]

    result = {
        "metadata": {"seed": SEED, "monte_carlo_seeds": mc_seeds, "dt_sec": DT, "python": platform.python_version(), "elapsed_sec": round(time.perf_counter() - t0, 3), "nees_95_bounds_6d": list(NEES_BOUNDS)},
        "sensor_validation": day1_sensor_metrics(),
        "constant_velocity": {
            "one_radar_ekf": {k: v for k, v in one_ekf.items() if k != "position_errors_m"},
            "three_radar_ekf": {k: v for k, v in three_ekf.items() if k != "position_errors_m"},
        },
        "coordinated_turn": {
            "ekf": {k: v for k, v in turn_ekf.items() if k != "position_errors_m"},
            "ukf": {k: v for k, v in turn_ukf.items() if k != "position_errors_m"},
        },
        "abrupt_maneuver": {
            "static_q": {k: v for k, v in maneuver_static.items() if k != "position_errors_m"},
            "adaptive_q": {k: v for k, v in maneuver_adaptive.items() if k != "position_errors_m"},
            "static_recovery_sec": recovery_time(maneuver_static["position_errors_m"], 25.0),
            "adaptive_recovery_sec": recovery_time(maneuver_adaptive["position_errors_m"], 25.0),
        },
        "degraded_sensing_monte_carlo": {
            "condition": "3 radars, 70% detection probability (30% dropout), 10% packet loss per radar",
            "coordinated_turn_ekf": summarize_runs(degraded_turn_runs),
            "abrupt_maneuver_adaptive_ekf": summarize_runs(degraded_maneuver_runs),
        },
    }
    out = Path(__file__).parent / "results" / "tracking_benchmark.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()


