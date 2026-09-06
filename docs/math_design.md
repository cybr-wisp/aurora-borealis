# Aurora Borealis — Mathematical design

## 1. State and process model

Aurora tracks a six-dimensional Cartesian state in a local ENU frame:

\[
x =
[x,\;y,\;z,\;v_x,\;v_y,\;v_z]^T.
\]

For a constant-velocity model over timestep \(\Delta t\),

\[
x_{k+1} = F(\Delta t)x_k + w_k
\]

with

\[
F =
\begin{bmatrix}
I_3 & \Delta t I_3 \\
0   & I_3
\end{bmatrix}.
\]

Unknown acceleration is represented as white process noise. For one spatial
axis with acceleration spectral density \(q\),

\[
Q_{1D} =
q
\begin{bmatrix}
\Delta t^3/3 & \Delta t^2/2 \\
\Delta t^2/2 & \Delta t
\end{bmatrix}.
\]

Aurora places one copy of this block on each position/velocity axis.

## 2. Radar measurement model

A sensor at \(s=[s_x,s_y,s_z]^T\) observes the relative position
\(d=[d_x,d_y,d_z]^T=p-s\).

The nonlinear measurement is

\[
h(x)=
\begin{bmatrix}
r\\
\theta\\
\phi
\end{bmatrix}
=
\begin{bmatrix}
\sqrt{d_x^2+d_y^2+d_z^2}\\
\operatorname{atan2}(d_y,d_x)\\
\operatorname{atan2}(d_z,\sqrt{d_x^2+d_y^2})
\end{bmatrix}.
\]

The EKF linearizes this model with the analytical Jacobian
\(H=\partial h/\partial x\). Velocity columns are zero because the instantaneous
radar observation depends only on position.

Innovation:

\[
v_k=z_k-h(\hat{x}_{k|k-1})
\]

with bearing/elevation residuals wrapped to \([-\pi,\pi)\).

Innovation covariance:

\[
S_k = H_kP_{k|k-1}H_k^T + R_k.
\]

Kalman gain:

\[
K_k=P_{k|k-1}H_k^TS_k^{-1}.
\]

Aurora solves the linear system through matrix factorization rather than
forming an explicit inverse.

## 3. UKF comparison

The UKF replaces first-order measurement linearization with deterministically
chosen sigma points around the predicted mean and covariance. Each point is
propagated through the nonlinear measurement function and recombined with
weights.

Aurora keeps EKF and UKF behind equivalent predict/update behavior so the
experimental suite can compare accuracy and consistency on coordinated turns.

## 4. Statistical consistency

For ground-truth state \(x_k\), estimate \(\hat{x}_k\), and covariance \(P_k\),

\[
\epsilon_k =
(x_k-\hat{x}_k)^TP_k^{-1}(x_k-\hat{x}_k).
\]

This is the normalized estimation error squared (NEES). For a consistent
six-dimensional Gaussian state estimate, NEES is compared with chi-squared
bounds for six degrees of freedom.

RMSE answers whether the estimate is accurate. NEES answers whether the
reported uncertainty is statistically credible. Aurora records both.

The normalized innovation squared (NIS),

\[
\nu_k=v_k^TS_k^{-1}v_k,
\]

is similarly used to monitor measurement-model inconsistency.

## 5. Maneuver adaptation

The nominal process model assumes approximately constant velocity. During
abrupt acceleration, innovation magnitude rises because the model is wrong.

Aurora monitors a window of NIS values. Persistent excessive innovation
temporarily scales \(Q\) upward, increasing process uncertainty and allowing
the filter to react more quickly. Once the innovation sequence returns toward
nominal behavior, the process-noise multiplier is removed.

This is adaptive process noise \(Q\); it is distinct from adaptive sensor noise
\(R\).

## 6. Data association

For an active track and candidate measurement, Aurora predicts the measurement
and computes innovation \(v\) and covariance \(S\).

The squared Mahalanobis distance is

\[
d^2=v^TS^{-1}v.
\]

Pairs outside a chi-squared gate are forbidden. The remaining costs form a
bipartite assignment matrix solved by the Hungarian algorithm.

The Hungarian stage is \(O(n^3)\) in the square problem size. Aurora therefore
benchmarks association latency as track count grows and separately
characterizes identity-switch failure around crossing targets.

## 7. Adaptive measurement-noise estimation

For one sensor, the empirical innovation covariance contains both predicted
state uncertainty and measurement uncertainty:

\[
E[vv^T] \approx HP_{pred}H^T + R.
\]

Therefore an innovation-based estimator is

\[
\hat{R}_k =
\frac{1}{N}\sum_{i=k-N+1}^{k} v_iv_i^T
-
\frac{1}{N}\sum_{i=k-N+1}^{k}H_iP_{i,pred}H_i^T.
\]

Aurora currently adapts the diagonal range/azimuth/elevation variances because
the configured radar model assumes independent measurement channels.

To keep the feedback loop stable, the estimate is:

1. computed over a finite window,
2. clamped relative to configured sensor variance,
3. exponentially smoothed,
4. maintained independently for each `sensor_id`.

A large learned/configured variance ratio marks that sensor as degraded.

## 8. Multi-sensor update semantics

At a shared timestamp, each track is predicted exactly once. Observations are
grouped by sensor and then sequentially associated and updated against the
latest posterior. Track lifecycle counters advance once per timestamp, not once
per contributing sensor.

This prevents a three-sensor frame from accidentally aging a track three times
while still allowing all three measurements to reduce uncertainty.
