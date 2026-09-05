# Day 5 — Multi-object association

Aurora's Day 5 tracker performs hard one-to-one data association in three
stages.

## 1. Statistical gating

For each predicted track and radar measurement, Aurora computes the innovation
and innovation covariance:

`v = z - h(x)`

`S = H P H^T + R`

The association cost is the squared Mahalanobis distance:

`d² = v^T S^-1 v`

Pairs outside the 95% chi-squared gate for a 3-dimensional radar measurement
are forbidden (`d² > 7.8147279`).

## 2. Hungarian assignment

The remaining track/measurement costs form a rectangular cost matrix. Aurora
pads the matrix with dummy assignments and runs a shortest augmenting-path
Hungarian solver. The implementation supports arbitrary rectangular matrices
and returns `-1` for tracks with no admissible measurement.

The benchmark reports p50/p95/p99 assignment latency at 5, 10, 25, 50 and 100
tracks.

## 3. Lifecycle

Tracks follow:

`TENTATIVE -> CONFIRMED -> COASTING -> DELETED`

Default policy:

- create tentative track on an unassigned measurement
- confirm after 3 consecutive associations
- coast after the first missed frame
- delete a confirmed track after 5 consecutive misses
- delete an unconfirmed tentative track after 2 misses

The pipeline predicts every track to the current frame, gates all admissible
pairs, solves the global assignment, applies EKF updates to matched tracks,
coasts unmatched tracks, and creates tracks for unmatched measurements.

## Failure boundary experiment

`experiments/crossing_phase_diagram.py` runs deterministic crossing-target
Monte Carlo trials without using ground-truth identity during assignment. The
ground-truth labels are used only after assignment to score whether a run
experienced an identity switch.

Outputs:

- `experiments/results/crossing_phase_diagram.csv`
- `docs/figures/crossing_phase_diagram.svg`

Run:

```powershell
python experiments/crossing_phase_diagram.py
.\build\Debug\bench_association.exe
```

Do not put example percentages in the README until this experiment has been
run; the committed values should be measured from Aurora's actual geometry and
noise configuration.
