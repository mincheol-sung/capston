"""path following 용 기준 시각 (2026-11-23, figure8_reference_node 의 path_follow_lead_s). ROS 에 의존하지 않는다.

왜: 지금 기준은 trajectory tracking — 기준 시각 t_ref 가 시계대로 흘러, 로봇이 마찰 때문에 늦으면 기준점이 앞서 달아나고
    뒤처짐이 그대로 위치 오차가 된다. 실차 계단 변조 8자 17 판 (analysis/torque_mpc_real/2026-11-23_load_real/contour_check.py)
    에서 위치 오차 제곱의 66~86 % 가 진행 방향 뒤처짐 (최대 −0.29 m), 경로 이탈 (contouring) 은 14~34 %.
    path following (Aguiar·Hespanha·Kokotović 2008 IEEE TAC 53(2)) 은 경로만 따라가면 되고 늦은 비용은 주행 시간으로 나온다.

방법 — 기준 시각에 상한·하한 (정식 용어 아님, 설명용 이름: "기준 시각 묶기")
  t_proj = 경로 위에서 로봇 (x, y) 에 가장 가까운 점의 기준 시각 (수선의 발, Serret–Frenet 경로 좌표의 호 길이에 해당)
  t_ref ← clamp(t_ref, t_proj, t_proj + lead)
    로봇이 따라오면 (t_ref − t_proj ≤ lead) 지금과 같은 trajectory tracking, 늦으면 기준이 t_proj + lead 에서 기다린다,
    로봇이 앞서면 기준을 로봇 자리로 당긴다. lead → 0 은 순수 path following, lead → ∞ 는 지금 동작.
  정사영 지점의 기준 속력이 v_min (0.1 m/s) 아래면 (출발·끝 정지 근처) 시계대로 둔다 — 거기서 묶으면 PP 는
  v = v_ref + k_along·e_along = 0 이라 출발을 못 한다 (baseline/pure_pursuit.py).
  정사영 지점부터 lead + DECEL_LOOK 안에 기준 감속이 있으면 위쪽 상한을 걸지 않는다 (시계대로) — MPC 는 기준 가속도를
  acc_dt 0.2 s 중앙 차분으로 앞먹임하는데, 감속 직전에 기준 시각을 묶으면 앞먹임이 "브레이크" 로 고정되고 위치 오차
  (앞으로) 와 맞서 몸체 힘 0 에 멈춘다 (시뮬 2026-11-23 path_follow/dbg: 끝점 6.5 cm 앞, F·U_s x −11.7 N, 140 s 정지).
  늦음은 가속·고속 선회에서 쌓이고 감속에서는 기준이 느려져 로봇이 따라잡으므로, 감속 구간을 시계대로 둬도 늦음이 다시 오차로 쌓이지 않는다.
  기준 속도·가속은 시각의 함수 그대로 꺼내므로 경로 위치별 속도 프로파일 v(s) 는 바뀌지 않는다.

정사영 찾기: 직전 t_proj 에서 −back ~ +fwd [s] 창 (8자 교차점에서 다른 바퀴 쪽으로 튀지 않게), 0.01 s 격자 → 0.001 s.
"""
import numpy as np

V_MIN = 0.1      # [m/s] 이 아래 기준 속력에서는 묶지 않는다 (출발·정지 근처)
DECEL_LOOK = 0.25  # [s] 정사영 지점 + lead 뒤로 이만큼 안에 감속이 있으면 묶지 않는다 (acc_dt/2 + 여유)


def project(xy_at, pose_xy, anchor, lo, hi, back=0.2, fwd=1.0, coarse=0.01, fine=0.001):
    """xy_at(t) → 경로 점 (2,). [max(lo, anchor−back), min(hi, anchor+fwd)] 에서 |xy_at(t) − pose_xy| 최소인 t."""
    a, b = max(lo, anchor - back), min(hi, anchor + fwd)
    if b <= a:
        return float(min(max(anchor, lo), hi))
    p = np.asarray(pose_xy, float)[:2]
    ts = np.arange(a, b + 1e-9, coarse)
    k = int(np.argmin([np.hypot(*(np.asarray(xy_at(x))[:2] - p)) for x in ts]))
    ts2 = np.arange(max(a, ts[k] - coarse), min(b, ts[k] + coarse) + 1e-9, fine)
    return float(ts2[int(np.argmin([np.hypot(*(np.asarray(xy_at(x))[:2] - p)) for x in ts2]))])


def decel_ahead(speed_at, t_proj, lead, look=DECEL_LOOK, step=0.02, tol=0.005):
    """speed_at(t) → 기준 속력. [t_proj, t_proj + lead + look] 에서 속력이 tol 넘게 줄어드는 곳이 있나."""
    v = [speed_at(x) for x in np.arange(t_proj, t_proj + lead + look + 1e-9, step)]
    return bool(np.any(np.diff(v) < -tol * step / 0.02))


def govern(t, t_proj, lead, v_proj, v_min=V_MIN, decel=False):
    """기준 시각 묶기 → (t_new, flag). flag +1 = 기다림 (로봇이 lead 보다 늦음), −1 = 앞당김 (로봇이 앞섬), 0 = 그대로.
    decel = 정사영 앞에 기준 감속이 있다 (decel_ahead) → 위쪽 상한을 걸지 않는다."""
    if v_proj < v_min:
        return t, 0
    if t > t_proj + lead and not decel:
        return t_proj + lead, 1
    if t < t_proj:
        return t_proj, -1
    return t, 0
