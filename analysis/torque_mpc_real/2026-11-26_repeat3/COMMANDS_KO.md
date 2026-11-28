# 15~20p 반복 시험 명령 (제어기별 3회차 추가)

- NUC에서 실행. `a` = arm, `s`/스페이스 = 정지, 회차가 끝나면 Ctrl+C.
- 초록 배너("측위 맞추기 끝: 맞음")가 나온 뒤에만 `a`. origin_mode 기본값 robot: arm한 위치·heading이 출발점.
- bag 위치: `~/capston/capston/bags/<날짜>/<시분초>_<이름>`
- 다음 회차 전에 이전 launch를 완전히 종료.
- 순서: 시나리오 안에서 MPC → PP → ADPP 를 1회차, 2회차, 3회차로 교대.
- 기본값과 같은 인자(controller:=mpc, disturbance:=true, use_w:=true, yaw_mode:=tangent)는 뺐다. d̂·w 항은 torque_mpc.yaml 기본값으로 켜진다.
- speed·accel: S1 max (launch 계산 0.61 m/s · 0.60 m/s²). S2 는 비우면 max. S3 polyline 은 launch 가 max 를 거부해 직선 가속 한계 1.15 m/s² 를 숫자로 (square_v_max 기본 0 = 부하 상한까지).
- S2의 MPC·PP 조건은 선택 회차(115609, 115422)와 다르다. 기존 1회 결과와 평균을 합치지 않는다.

## 시나리오 1: circles8 (슬라이드 15·16)

### 1회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_mpc_r1
```

```bash
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_pp_r1
```

```bash
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_adpp_r1
```

### 2회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_mpc_r2
```

```bash
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_pp_r2
```

```bash
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_adpp_r2
```

### 3회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_mpc_r3
```

```bash
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_pp_r3
```

```bash
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=max accel:=max max_track_error:=0.5 bag:=s1_adpp_r3
```

## 시나리오 2: zigzag (슬라이드 17·18)

### 1회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_mpc_r1
```

```bash
amrall use_mpc:=true controller:=pp pp_lookahead_min:=0.1 pp_lookahead_max:=0.1 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_pp_r1
```

```bash
amrall use_mpc:=true controller:=adpp pp_lookahead_min:=0.1 pp_lookahead_max:=0.2 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_adpp_r1
```

### 2회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_mpc_r2
```

```bash
amrall use_mpc:=true controller:=pp pp_lookahead_min:=0.1 pp_lookahead_max:=0.1 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_pp_r2
```

```bash
amrall use_mpc:=true controller:=adpp pp_lookahead_min:=0.1 pp_lookahead_max:=0.2 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_adpp_r2
```

### 3회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_mpc_r3
```

```bash
amrall use_mpc:=true controller:=pp pp_lookahead_min:=0.1 pp_lookahead_max:=0.1 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_pp_r3
```

```bash
amrall use_mpc:=true controller:=adpp pp_lookahead_min:=0.1 pp_lookahead_max:=0.2 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 bag:=s2_adpp_r3
```

## 시나리오 3: polyline_stop (슬라이드 19·20)

### 1회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_mpc_r1
```

```bash
amrall use_mpc:=true controller:=pp shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_pp_r1
```

```bash
amrall use_mpc:=true controller:=adpp shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_adpp_r1
```

### 2회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_mpc_r2
```

```bash
amrall use_mpc:=true controller:=pp shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_pp_r2
```

```bash
amrall use_mpc:=true controller:=adpp shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_adpp_r2
```

### 3회차

```bash
amrall use_mpc:=true limit_mode:=friction shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_mpc_r3
```

```bash
amrall use_mpc:=true controller:=pp shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_pp_r3
```

```bash
amrall use_mpc:=true controller:=adpp shape:=polyline square_corner_mode:=stop accel:=1.15 max_track_error:=0.5 bag:=s3_adpp_r3
```

---

## PPT 15~20쪽 기록 그대로 재현 (2026-11-26 확인)

PPT 회차 확정: S2 는 슬라이드에 번호가 적혀 있음. S1·S3 은 후보 bag 의 구동·제동·순 일을 다시 계산해 PPT 표 값과 0.1~0.2 J 안에서 맞는 회차로 정함.
인자는 각 bag 의 run_info/args.txt, 그 뒤 바뀐 기본값은 run_info/params 덤프와 지금 코드 비교로 확인.

| 시나리오 | 제어기 | PPT 회차 |
|---|---|---|
| S1 circles8 | MPC / PP / ADPP | 145703_c8d2_mpcw_r1 / 145812_c8d2_pp_r1 / 145912_c8d2_pp_r1(이름만 pp, 실제 adpp) |
| S2 zigzag | MPC / PP / ADPP | 115609 / 115422 / 224008 |
| S3 polyline | MPC / PP / ADPP | 152823_sq2_a20_mpcw_r2 / 153213_sq2_a20_pp_r3 / 151817_sq2_a20_adpp_r1 |

지금 코드에서 당시와 달라진 기본값 (명령에 넣어 되돌림):
- MPC `mpc_roller_rr` 0 → 0.88, `mpc_inertia_z` 0.06 → 0.03 (2026-11-26 17:52 변경·빌드) → `mpc_roller_rr:=0.0 mpc_inertia_z:=0.06`
- polyline `square_corner_mode` 당시 stop → 지금 pass → `square_corner_mode:=stop`
- 구동 방향 보정 `torque_plant.yaml drive` 당시 0·0 → 지금 3.1°·0.8 °/(m/s²) (2026-11-26 18:40, 세 제어기 공통) → `drive_yaw_deg:=0 drive_slip_coef:=0`

```
amrall use_mpc:=true limit_mode:=friction velocity_source:=wheels mpc_roller_rr:=0.0 mpc_inertia_z:=0.06 shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p1_mpc_r1
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p1_pp_r1
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p1_adpp_r1

amrall use_mpc:=true mpc_roller_rr:=0.0 mpc_inertia_z:=0.06 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p2_mpc_r1
amrall use_mpc:=true controller:=pp pp_lookahead_min:=0.1 pp_lookahead_max:=0.2 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p2_pp_r1
amrall use_mpc:=true controller:=adpp pp_lookahead_min:=0.1 pp_lookahead_max:=0.2 shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p2_adpp_r1

amrall use_mpc:=true limit_mode:=friction velocity_source:=wheels mpc_roller_rr:=0.0 mpc_inertia_z:=0.06 shape:=polyline square_corner_mode:=stop accel:=2.0 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p3_mpc_r1
amrall use_mpc:=true controller:=pp shape:=polyline square_corner_mode:=stop accel:=2.0 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p3_pp_r1
amrall use_mpc:=true controller:=adpp shape:=polyline square_corner_mode:=stop accel:=2.0 drive_yaw_deg:=0 drive_slip_coef:=0 bag:=p3_adpp_r1
```
