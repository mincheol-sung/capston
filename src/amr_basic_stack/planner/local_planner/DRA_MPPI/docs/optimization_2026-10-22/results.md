# 5 Hz regression fix — 2026-10-22

후속 사용자 요청으로 YAML은 기존 `monte_carlo`/밀도124 예산으로 복원했다.
아래 `gaussian_disk` 측정은 당시 설정의 기록이며 현재 기본 설정의 측정이 아니다.

## Cause and change

The previous experimental YAML raised every rectangle MC density from 124 to 4096
and guide/final minimum samples from 128/256 to 4096. This dominated runtime despite
reducing K to 100. A 180 ms timeout then rejected many cycles that would have
finished before the 200 ms callback deadline. That timeout is still 180 ms.

With **identical candidate SHA256**, 10 obstacles, 2 modes, N=10, dt=.2,
static checks ON and 2 workers, the kernel ablation measured:

| Covariance | K | Original MC budget mean / p99 ms | High-density MC mean / p99 ms | Direct disk mean / p99 ms |
|---|---:|---:|---:|---:|
| Isotropic | 100 | 25.07 / 26.97 | 232.21 / 235.98 | 16.05 / 17.37 |
| Isotropic | 500 | 29.28 / 29.73 | 388.12 / 393.71 | 22.79 / 22.94 |
| Rotated anisotropic | 100 | 24.90 / 25.44 | 223.33 / 225.49 | 22.41 / 24.08 |
| Rotated anisotropic | 500 | 29.62 / 30.91 | 383.62 / 389.67 | 55.56 / 56.38 |

These are short kernel comparisons, not ROS p99 claims. Baseline: 2 warmups + 8
measured cycles; new solver: 2 + 5. Candidate hashes agree across profiles.
Replacing only the final integration still took 324.63 ms mean / 333.89 ms p99
for K500 anisotropic; the primary integration also needed replacement.
See [baseline raw](baseline/kernel_raw.csv), [new raw](disk_pruned/kernel_raw.csv).

Production YAML now selects `dynamic_risk.mode: gaussian_disk`, restores **K=500**,
and retains N=10, dt=.2, 5 Hz, epsilon=.05, 2 workers, static swept collision,
final representative re-rollout/re-cost/threshold checks, and the .18 s timeout.
Physical limits, costs, switch penalty and hysteresis are unchanged by this fix.
`monte_carlo` remains selectable with its preserved high-precision comparison
budgets. This change evaluates the same Gaussian disk probability by a different
numerical method; it does not claim to execute rectangle Monte Carlo in the new mode.

## Numerical method

For every obstacle/mode/timestep/candidate, covariance is regularized using the
same eigenvalue floor as before. Isotropic normals use the noncentral chi-square
CDF for the disk mass. General normals integrate the conditional Y-normal CDF
against the X marginal with `x = R sin(theta)`, Gauss-Legendre orders 16→32→64→128→256→512.
A minimum order proportional to radius / narrowest sigma prevents agreement of
two undersampled grids from accepting a missed narrow peak. Acceptance requires
successive orders within 1e-8; add 4× their difference + 5e-8 to anisotropic mode
mass. If unresolved/nonfinite, mode mass is 1. Refinement is an error estimate,
**not a certified error bound**. Diagnostic counters expose analytic calls,
quadrature-node evaluations, unresolved integrals and refinement differences.
MC sample counts are zero in this mode because no MC samples are drawn.

For a Gaussian mean at distance d>R, projection onto the mean-to-center axis gives
a half-space containing the disk. Its Gaussian tail upper-bounds disk mass. If
that bound ≤1e-10, retain the upper bound as the mass instead of doing quadrature.
No finite-distance cutoff assigns a zero probability to a distant obstacle.
Mixture modes are weighted first, then independent obstacles combine as
`1 - product(1 - p_obstacle)`, exactly as in the previous probability model.
Each candidate's integral is independent of the extent of the other candidates.

[Independent accuracy check](accuracy/summary.json): 384 threshold-near cases,
64 random Gaussian mixtures, 1/2/5/20 obstacles, rotated axes, sigma .08–.6,
seed 80117. Reference: separate polar radial/angle quadrature (80 vs 160 nodes).
Maximum absolute error **9.31e-7**, p95 **8.62e-7**, MAE **2.86e-7**;
reference refinement difference ≤1.34e-15; observed false-safe **0/192** unsafe
cases, false-unsafe 0, unresolved 0. Raw data: [accuracy_raw.csv](accuracy/accuracy_raw.csv).
This suite does not certify arbitrary distributions or predictor calibration.

## ROS measurements

ROS Jazzy, isolated localhost domain 173, actual planner + synthetic DDS inputs +
subscriber, production .18 s timeout, static map ON, command publishing OFF.
K=500, 20 obstacles ×2 modes, all rotated anisotropic covariance
`[[.05,.025],[.025,.025]]`, live warm-starts (not frozen). All actual parameters,
source hashes, environment, telemetry, callback/receive raw records are saved per run.
The robot odometry is held fixed by the fixture; selecting a moving reference
is not a physical robot-driving test. No perception/MPC/driver load is included.

| Run | Measured cycles | Callback p99 / max | Timeout | >200 ms |
|---|---:|---:|---:|---:|
| [Short](ros_short20_anisotropic/summary.json) | 100 after 10 warmup | 129.46 / 132.98 ms | 0 | 0 |
| [Stopped early](ros_long20_anisotropic/summary.json) | 1297 after 50 warmup | 126.20 / 136.89 ms | 0 | 0 |

Short scheduled-time→reference-publish p99 133.37 ms; publish interval
p99 221.92 ms / max 222.65 ms, average 200.01 ms. Passing callback deadlines does
not imply publication intervals are bounded by 200 ms: variable completion time
causes jitter around the timer period. All 100 measured cycles selected a moving
guide; no primary/final unresolved integrals. No commands were sent to actuators.
Timer schedule phase is estimated from the monotonic initial phase, not kernel tracing.

Long run was stopped early to finish promptly after the user's latency complaint;
**3000 cycles were not completed**. 1296 saved measured cycles completed normally and selected moving guides;
one additional cycle was interrupted by SIGINT (recorded as `exception`). Recorder: 1347 rows, 0 drops.
Partial-run publication interval p99 219.03 ms,
maximum 228.46 ms. SIGINT caused a ROS already-shutdown
exception after recorder flush; timing CSV and summary were reconstructed from the raw rows.

## Functional checks and remaining limits

- pytest and colcon: **188 passed**, 0 errors/failures/skips.
- Parallel workers 1/2/5 yield exactly equal candidate CP, final CP and selected
  controls for the deterministic integrator. Tests cover CDF/tail accuracy,
  rotated mixtures, per-obstacle radii/weights, and unresolved fail-closed behavior.
- Offline kinematic suite, 20 scenes ×1 seed, K500 each strategy: guide **16/20**,
  standard **15/20**, observed collisions 0 for both. Full per-scene results and
  limits: [closed_loop_summary.json](navigation/closed_loop_summary.json).
  Timeout is 10 s only in this offline convergence test. This does not validate
  real MPC tracking or real collision safety, nor establish strategy superiority.
- Gaussian covariance/mixture calibration and dependence between obstacles are
  outside this numerical regression fix. Full autonomy under perception/MPC CPU
  load and multiple 3000-cycle repetitions across all conditions remain untested.

## Reproduction

From this package directory, source `/opt/ros/jazzy/setup.bash` and workspace
`install/setup.bash`, export `PYTHONPATH=.:$PYTHONPATH` and set BLAS/OMP threads to 1.

```bash
python3 tools/benchmark_risk_optimization.py --output /tmp/disk_ablation
python3 tools/validate_gaussian_disk.py --output /tmp/disk_accuracy
python3 tools/validate_ros_pilot.py --output /tmp/disk_ros20 \
  --cycles 3000 --warmup 50 --obstacles 20 --covariance anisotropic
```

Before-change source is preserved in `before_optimization.tar.gz`. Previous
high-density failure reports remain under `../validation_2026-10-22`.
