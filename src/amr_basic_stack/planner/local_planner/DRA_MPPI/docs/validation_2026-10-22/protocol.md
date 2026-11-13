# 2026-10-22 pre-tuning protocol

Recorded before this run's MC sweep/tuning. Synthetic numerical acceptance is
MAE <= 0.002, absolute error p95 <= 0.005, maximum <= 0.015 in operational
candidate domains (span <= 2 m). Epsilon stays 0.05. Report false-safe denominator
as reference > epsilon, and separately reference >= epsilon + 0.01; require zero
observed false-safe in the latter group. Near-threshold errors are reported,
not a guarantee of safe classification. Errors of .005/.01 are respectively
10%/20% of the unchanged threshold, a numerical engineering criterion, not an
approved physical collision-risk specification. Extreme span=50 m is stress
and must reject insufficient integration rather than claim safety.

Reference: isotropic noncentral chi-square CDF and independent polar
Gauss-Legendre integration for anisotropic mixtures, checked at two orders.
Tuning seeds 0..7; held-out accuracy seeds 100..115 and navigation seeds 101..103.
All timings use perf_counter/monotonic and NumPy linear quantiles. Do not pool
conditions or average p99 for acceptance. Full protocol target is 3000 post-warmup
ROS cycles x 3 repeats per A/B/C/D and per representative load. Shorter runs
must be labelled pilots with counts. A = batch/common MC, B = split/1 worker,
C = split/2, D = split/5; all retain K=500,N=10,dt=.2, static/final checks.
B/C/D have identical guide seeds; A/B compare numerical error, not equal MC bits.
No hardware arming, motor nodes, or operational-domain references are allowed.

Post-evaluation note (criteria unchanged): density 2048 passed the tuning seeds
but failed held-out seeds 100..115 at maximum absolute error 0.01838. It was
rejected. Density 4096 was checked again on new seeds 200..215 with the actual
4096 floor/131072 cap. Actual-candidate common/split comparisons use seeds
200..207 and independent ncx2 references; near-epsilon false-safe is reported.
