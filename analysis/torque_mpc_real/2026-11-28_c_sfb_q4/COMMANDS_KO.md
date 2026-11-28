# 새 C (슬립 되먹임 lqr 1.0 + W ×4) 로 11-26 세 경로 다시 돌리기

- 바꾼 것 (11-26 C 대비) 세 가지만:
  - 슬립 되먹임 (바퀴 각속도 P 되먹임) Δτ = −K_ω (ω − J x_e): `slip_fb:=true slip_fb_gain:=lqr`
  - LQR 배율: `slip_fb_scale:=1.0` (K_ω 대각 0.0162 N·m/(rad/s))
  - Q: `mpc_W:=80,80,10,2,2,1` (11-26 은 20,20,10,2,2,1)
  - 셋 다 지금 `torque_mpc.yaml` 기본값과 같지만, yaml 이 또 바뀌어도 조건이 남게 명령에 적었다.
- 나머지는 11-26 회차의 `run_info/args.txt` 그대로. `run_info/params/torque_mpc_node.yaml` 을 11-28 155544 와 diff 해서 확인:
  - S2·S3 (11-26 19:05 이후): 위 세 가지 말고 차이 없음.
  - S1 (11-26 17:26 `172646_s1_mpc_r1`): 그 뒤 기본값이 더 바뀌었다 — `mpc_roller_rr` 0 → 0.88 · `mpc_inertia_z` 0.06 → 0.03 (11-26 17:52),
    `drive_yaw_deg` 0 → 3.1 · `drive_slip_coef` 0 → 0.8 (11-26 18:40). 사용자 결정으로 되돌리지 않고 지금 기본으로 돌리며, PP·ADPP 도 다시 돈다 (S1 절).
- 띄우는 법: NUC, 초록 배너 ("측위 맞추기 끝: 맞음") 뒤에만 `a`, 정지 `s`/스페이스, 회차 끝나면 Ctrl+C 로 완전히 내린 뒤 다음 회차.
  띄우기 전 `ros2 node list | sort | uniq -d` 가 비어 있어야 한다.
- bag: `~/capston/capston/bags/2026-11-28/<시분초>_<이름>`. 이름 = 11-26 이름에 `sfbq4` 를 붙여 구분.

## 자동 순서로 돌리기 (2026-11-28 추가, 사람 없이) — 아래 회차별 명령 대신

launch 한 번으로 세 경로 × 세 제어기를 다 돈다 (amr/notes_amr/pitwall_usage.md "세 제어기 자동 순서"). 경로 조건은 `pit s1|s2|s3` 프리셋 =
아래 명령과 같은 11-26 값 (기준 회차 params 덤프에서 옮김) + 새 C MPC 값. 세 제어기 모두 지금 기본값 (롤러·구동 방향 보정 켬).

```bash
amrall3 bag:=seq_sfbq4
```

초록 배너 뒤, 로봇을 출발점에 두고 다른 터미널에서:

```bash
pit s1 && amrseq
```

```bash
pit s2 && amrseq
```

```bash
pit s3 && amrseq
```

- 회차 수: 세 경로 모두 제어기마다 5 (15 회차, 2026-11-28 사용자 결정 — 기본값). s2 zigzag 는 + 180° 복귀 회전 15 번, fwd·rev 가 제어기마다 3:2 또는 2:3, 끝은 A 반대편 B. 실제로 돈 것: s2·s3 6 바퀴, s1 5 바퀴. 바꾸려면 `amrseq --n 6`.
- 회차 사이에 사람이 할 일 없음. fault·사람 정지·abort 면 순서 전체가 멈추고 다시 무장하지 않는다 → 출력의 `--skip`·`--anchor-pose` 로 이어서.
- 확인: `amrruns bags/2026-11-28/*_seq_sfbq4` 가 판마다 제어기와 다리 (data/return, fwd/rev) 를 보여 준다.
- 아래 회차별 명령은 손으로 돌릴 때용으로 남긴다.

## S1 circles8 — 세 제어기 모두 지금 기본값 (2026-11-28 사용자 결정)

- 경로 조건은 11-26 회차 (172646 MPC · 172737 PP · 172939 ADPP) 그대로. 되돌리는 인자 4 개는 **안 넣는다** → 롤러 0.88 N · I_z 0.03 · 구동 방향 3.1° · 0.8 °/(m/s²).
- 구동 방향 보정은 PP·ADPP 의 `wheel_pi` 도 읽는다 → 11-26 S1 PP·ADPP (보정 0) 와 견주면 A 대비 C 차이에 보정 몫이 섞인다. 그래서 세 제어기를 다 새로 돌린다.
- 11-26 S1 MPC 와는 바뀐 것이 7 개 (위 3 + 롤러·I_z·구동 2) — 새 C 세 변경의 효과만 따로 가를 수는 없다.
- 순서: 회차 안에서 MPC → PP → ADPP 교대 (11-26 과 같음).

### 1회차

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s1_sfbq4_r1
```

```bash
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 bag:=s1_pp_r1
```

```bash
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 bag:=s1_adpp_r1
```

### 2회차

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s1_sfbq4_r2
```

```bash
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 bag:=s1_pp_r2
```

```bash
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 bag:=s1_adpp_r2
```

### 3회차

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s1_sfbq4_r3
```

```bash
amrall use_mpc:=true controller:=pp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 bag:=s1_pp_r3
```

```bash
amrall use_mpc:=true controller:=adpp shape:=circles8 width:=2.0 crossing_blend_m:=0.6 speed:=1.0 accel:=1.0 max_track_error:=0.5 bag:=s1_adpp_r3
```

## S2 zigzag (11-26 기준 회차: 190531 · 191148 · 200634 `s2zz_mpc`)

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s2zz_sfbq4_r1
```

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s2zz_sfbq4_r2
```

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=zigzag zigzag_n:=9 zigzag_angle_deg:=30 zigzag_profile:=square zigzag_reverse:=false zigzag_corner:=pass max_track_error:=0.5 slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s2zz_sfbq4_r3
```

## S3 polyline (11-26 기준 회차: 201259 · 201604 · 201944 `s2pl_mpc`)

- 11-28 `155209_stress15` 가 이미 이 조건 (인자·파라미터 같음) 의 1 회차다. `poly` 별칭과도 같다.

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=polyline accel:=1.5 square_v_max:=1.5 square_corner_mode:=pass slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s2pl_sfbq4_r1
```

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=polyline accel:=1.5 square_v_max:=1.5 square_corner_mode:=pass slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s2pl_sfbq4_r2
```

```bash
amrall use_mpc:=true controller:=mpc limit_mode:=friction shape:=polyline accel:=1.5 square_v_max:=1.5 square_corner_mode:=pass slip_fb:=true slip_fb_gain:=lqr slip_fb_scale:=1.0 mpc_W:=80,80,10,2,2,1 bag:=s2pl_sfbq4_r3
```

## 회차 뒤 확인

```bash
grep -H -A6 "^    W:" ~/capston/capston/bags/2026-11-28/*_sfbq4_r*/run_info/params/torque_mpc_node.yaml
grep -H "slip_fb_enable\|slip_fb_gain\|slip_fb_scale\|roller_rr\|inertia_z\|drive_yaw_offset_deg" ~/capston/capston/bags/2026-11-28/*_sfbq4_r*/run_info/params/torque_mpc_node.yaml
```

- 기대값 (MPC 회차): `slip_fb_enable: true` · `slip_fb_gain: lqr` · `slip_fb_scale: 1.0` · W [80, 80, 10, 2, 2, 1] · `roller_rr: 0.88` · `inertia_z: 0.03` · `drive_yaw_offset_deg: 3.1`.
- S1 PP·ADPP 회차는 `grep -H drive_yaw_offset_deg ~/capston/capston/bags/2026-11-28/*_s1_*pp_r*/run_info/params/torque_output_node.yaml` → 3.1.
