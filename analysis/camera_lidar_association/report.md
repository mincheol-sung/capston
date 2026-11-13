# Camera–LiDAR association bag analysis

Analyzed all three `mpc_chain_20260923_*` bags without changing their contents.

## Recorded associations

Denominator: camera-origin object observations in synchronized `/detections_fused` frames, sources 0 and 2. Numerator: source 2. Repeated observations of one object count separately. This is association coverage, not ground-truth precision or recall.

| Bag suffix | Camera observations | Fused observations | Match rate | Person fused/observations | Person match rate |
|---|---:|---:|---:|---:|---:|
| 113151 | 3169 | 109 | 3.44% | 5/847 | 0.59% |
| 135325 | 1516 | 115 | 7.59% | 4/609 | 0.66% |
| 135606 | 5084 | 239 | 4.70% | 42/1831 | 2.29% |
| Total | 9769 | 463 | 4.74% | 51/3287 | 1.55% |

## Reconstruction and spatial consistency

Camera frames were recovered by exact header stamp. Paired LiDAR stamps were recovered from `/lidar_obstacles` published alongside fusion, using the closest bag receipt timestamp within 20 ms. This recovered 7613/7616 fusion frames. Current default yaw/representative-point corrections, covariance models, gate 9.21 and Hungarian assignment reproduced 100% of source decisions on recovered observations. All 463 recorded fused outputs also agreed with reconstructed fused positions within 10 micrometers. This strongly supports the recovered associations and calibration for these data; it does not prove object identity correctness.

| Bag | Raw camera–LiDAR separation median | Corrected separation median | Corrected separation P95 | Maximum |
|---|---:|---:|---:|---:|
| 113151 | 17.81 cm | 2.94 cm | 9.18 cm | 11.58 cm |
| 135325 | 3.30 cm | 1.15 cm | 2.58 cm | 17.12 cm |
| 135606 | 17.96 cm | 3.29 cm | 6.05 cm | 11.21 cm |

These separations are between accepted observations, not errors against ground truth. Gating selects small residuals by construction. No accepted camera detection had two LiDAR candidates inside the gate; this means no ambiguity under the configured gate, not zero false associations.

## Unmatched person observations

| Bag | Recovered unmatched person observations | Nearest corrected LiDAR within 20 cm | Within 50 cm | Median nearest distance | Median best chi-square |
|---|---:|---:|---:|---:|---:|
| 113151 | 842 | 179 (21.3%) | 647 (76.8%) | 35.2 cm | 1312 |
| 135325 | 604 | 113 (18.7%) | 281 (46.5%) | 72.6 cm | 5049 |
| 135606 | 1789 | 449 (25.1%) | 1235 (69.0%) | 28.4 cm | 1358 |

The nearest spatial candidate need not be the same physical object, and the candidate with minimum chi-square can differ from the nearest Euclidean candidate. The large costs compared with gate 9.21 show a strong discrepancy between person observations and the current spatial/noise model. One unmatched person observation in 135325 could not be reconstructed.

## Timing and robot motion

Camera–LiDAR timestamp separation median is 16.8–17.1 ms, P95 approximately 32 ms, maximum 66.6 ms. Fusion produces approximately 10 Hz from approximately 15 Hz camera input; the roughly two-thirds frame ratio is consistent with the LiDAR input rate and must not be interpreted as an association failure rate.

For 135606, recorded match rate is 6.38% when the robot is stationary and 2.90% when moving. Stationary means odometry planar speed <0.05 m/s and absolute yaw rate <0.05 rad/s, using the nearest odometry header timestamp. For 135325 rates are 13.87%/4.38%; 113151 reverses this pattern (0.99%/3.70%). Scene composition is uncontrolled, so these comparisons cannot establish motion as the cause. Current fusion code does not compensate observation epoch differences for ego or object motion.

## Interpretation and next investigation

Association coverage is low, especially for camera-labeled people. Accepted pairs are spatially consistent with the calibration, but true precision and recall cannot be measured without labels identifying objects visible to both sensors. Camera-only detections can be legitimate: the low 2D LiDAR scan plane, occlusion and different representative points prevent some camera objects from having a corresponding LiDAR cluster.

Prioritize inspecting unmatched person observations alongside their LiDAR clusters in a common motion-compensated frame. Separate objects intersecting the scan plane from objects with no valid LiDAR observation; inspect depth representative points, segmentation, extrinsic calibration and temporal offsets. Only then refit person-specific residual/covariance models and assess gate changes against labeled positive and negative pairs. Broadening the gate alone does not establish improvement.

Machine-readable results: `results.json` and per-bag `*_unmatched.json`. Reproducible read-only script: `analyze.py`; source ROS Jazzy and the workspace install before running.
