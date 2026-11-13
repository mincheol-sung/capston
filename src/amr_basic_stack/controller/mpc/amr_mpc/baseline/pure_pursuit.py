#!/usr/bin/env python3
"""퓨어 퍼슛 (비교군 A 의 경로 추종기) — 시간 기준궤적을 따라가는 판.  ROS 에 의존하지 않는다.

입력:  기준열 (core/reference.py ReferenceHorizon — figure8 의 지도 고정 8자), 지금 시각 t, 로봇 포즈 [x, y, yaw]
출력:  몸체 속도 명령 (v_x, ω_z).  v_y = 0 — 차동 구동처럼 헤딩을 경로 접선에 맞춰 달린다 (헤딩 = 접선 추종).

교과서 퓨어 퍼슛 (Coulter 1992)
    로봇에서 L_d 떨어진 경로 위의 점(룩어헤드 점)을 몸체 좌표 (x_l, y_l) 로 보고, 그 점을 지나는 원호의 곡률
        κ = 2·y_l / L²,   L² = x_l² + y_l²
    을 따라 ω_z = v·κ 로 돈다.  원 위를 정확히 달리고 있으면 κ = 1/R 이 그대로 나온다 (앞먹임이 따로 필요 없다).
    룩어헤드 거리는 속도에 비례:  L_d = clamp(lookahead_time·|v_ref|, lookahead_min, lookahead_max).

시간 기준궤적이라 바꾼 것 두 가지 (뒤처짐 대책) ─────────────────────────────────────────────────
  교과서 판은 "경로 위에서 로봇과 가장 가까운 점" 부터 L_d 앞을 본다. 그 판은 모양만 따라가고 시간은 모른다.
  진행 속도를 기준 속도 v_ref 로 두면, 시작 지연·코너 안쪽 지름길·감속 구간에서 생긴 시간 차가 영영 안 메워져
  위치 오차(= 기준 시각의 점과의 거리)가 계속 남는다. 8자는 B·C 와 같은 "시각별 위치" 로 평가하므로 그대로는 불공정하다.

  ① 룩어헤드 탐색의 출발점 = 기준 시각 t 의 점 (가장 가까운 점이 아니라).
     그 점부터 경로를 앞으로 걸어가며 로봇에서 L_d 이상 떨어진 첫 점(원과 경로의 교점, 선형 보간)을 고른다.
       · 로봇이 뒤처져 기준점이 L_d 밖에 있으면 기준점 자체를 겨눈다 → 경로 모양보다 "제 시각 위치" 로 향한다.
       · 로봇이 앞서 있으면 기준점이 L_d 원 안이라 원 밖의 첫 점을 겨눈다 → 교과서 판과 같아진다.
     가장 가까운 점 탐색이 필요 없어 8자 교차점에서 다른 가지로 건너뛰는 일도 없다 (시간이 가지를 정한다).
  ② 진행 속도 v = v_ref + k_along·e_along   (e_along = 기준점이 로봇 헤딩 방향으로 앞선 거리 [m])
     뒤처지면 빨리, 앞서면 천천히 — 시간 오차에 대한 P 제어. k_along = 0 이면 교과서처럼 v = v_ref.
     주행 중(v_ref > 0)에는 후진하지 않는다 (v ≥ 0): 퓨어 퍼슛 기하는 전진을 가정하고, 앞섰을 때는 서서 기다리면 된다.
  둘 다 Kanayama(1990) 류 시간 추종 제어기의 진행 방향 항과 같은 생각이다. 차이는 측면·헤딩 보정을 여전히
  퓨어 퍼슛 기하(κ = 2y_l/L²) 하나로 한다는 것 — 비교군 A 를 "퓨어 퍼슛" 으로 남기기 위해서다.

제자리 회전 구간 (8자 앞뒤 정렬 회전, 정지 기준)
  v_ref ≈ 0 이면 룩어헤드 점이 정의되지 않는다 (경로가 앞으로 안 나간다). |v_ref| < rotate_speed 면
      ω = ω_ref + k_heading·(θ_ref − θ),   v = k_along·e_along (앞뒤 위치만 붙잡는다, 후진 허용)
  로 바꾼다. 측면 오차는 이 구간에서 못 고친다 (v_y = 0 이므로) — 비교군 A 의 구조적 한계로 둔다.
"""
from dataclasses import dataclass
import math

import numpy as np

from amr_mpc.core.reference import wrap


@dataclass
class PurePursuitConfig:
    lookahead_time: float = 0.5     # [s]  L_d = 이 값 × |v_ref|
    lookahead_min: float = 0.15     # [m]
    lookahead_max: float = 0.60     # [m]
    k_along: float = 1.0            # [1/s] 진행 방향 시간 오차 보정 (0 = 교과서 퓨어 퍼슛)
    k_heading: float = 2.0          # [1/s] 제자리 회전 구간 헤딩 보정
    rotate_speed: float = 0.02      # [m/s] |v_ref| 가 이보다 작으면 제자리 회전 구간
    v_max: float = 1.0              # [m/s]
    w_max: float = 3.0              # [rad/s]
    search_dt: float = 0.01         # [s] 룩어헤드 탐색 시간 간격 (기준 시각부터 앞으로 걷는다)
    search_horizon: float = 3.0     # [s] 그 이상은 안 본다 (기준열 끝 4 s 안)

    @classmethod
    def from_dict(cls, d):
        names = cls.__dataclass_fields__.keys()
        return cls(**{k: float(v) for k, v in d.items() if k in names})


def lookahead_distance(v_ref, cfg):
    """L_d = clamp(lookahead_time·|v_ref|, min, max)."""
    return float(np.clip(cfg.lookahead_time * abs(v_ref), cfg.lookahead_min, cfg.lookahead_max))


def _circle_cross(p0, p1, c, L):
    """선분 p0→p1 위에서 |p − c| = L 이 되는 첫 점 (p0 는 원 안, p1 은 원 밖이라고 본다)."""
    d = p1 - p0
    f = p0 - c
    a = float(d @ d)
    if a < 1e-12:
        return p1
    b = 2.0 * float(f @ d)
    cc = float(f @ f) - L * L
    disc = max(0.0, b * b - 4.0 * a * cc)
    s = (-b + math.sqrt(disc)) / (2.0 * a)
    return p0 + float(np.clip(s, 0.0, 1.0)) * d


def find_lookahead(ref_at, t, pos, L, cfg):
    """기준 시각 t 의 점부터 경로를 따라 앞으로 걸어가며, pos 에서 L 이상 떨어져 있고 **멀어지는 중인** 첫 점 [x, y].
      · 기준점이 원 안          → 경로가 원 밖으로 나가는 교점 (교과서 퓨어 퍼슛)
      · 기준점이 원 밖·멀어짐    → 기준점 그 자체 (뒤처짐 — 제 시각 위치를 겨눈다)
      · 기준점이 원 밖·다가옴    → 로봇이 L 이상 앞서 있다. 기준점은 로봇 뒤라 겨누면 돌아선다 → 건너뛰고
                                  원을 지나 나가는 교점(원에 안 들어오면 가장 가까운 점)
    "멀어지는 중" 조건이 없으면 마지막 경우에 뒤쪽 기준점을 골라 로봇이 뒤로 돈다 (단위 시험으로 확인, 2026-11-11).
    끝까지 못 찾으면(경로가 멈춤·짧음) 마지막으로 본 점. 반환 (점, 그 점의 기준 시각 − t)."""
    pos = np.asarray(pos, float)[:2]
    n = max(1, int(round(cfg.search_horizon / cfg.search_dt)))
    prev, d_prev = None, None
    for i in range(n + 1):
        p = np.asarray(ref_at(t + i * cfg.search_dt)[0][:2], float)
        d = float(np.hypot(*(p - pos)))
        if prev is not None and d >= L and d >= d_prev:
            if d_prev < L:
                return _circle_cross(prev, p, pos, L), i * cfg.search_dt
            return prev, (i - 1) * cfg.search_dt          # 앞 점도 원 밖이고 여기서부터 멀어진다 → 앞 점
        prev, d_prev = p, d
    return prev, n * cfg.search_dt


def pure_pursuit(cfg, ref_at, t, pose):
    """한 번 계산. ref_at(t) → (Xr[3], ur[3]) (ReferenceHorizon.at), pose = [x, y, yaw].
    반환 (v_x, ω_z, info dict)."""
    x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    Xr, ur = ref_at(t)
    v_ref = float(ur[0])
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = float(Xr[0]) - x, float(Xr[1]) - y
    e_along = c * dx + s * dy                       # 기준점이 로봇 앞(+)
    e_cross = -s * dx + c * dy                      # 기준점이 로봇 왼쪽(+)
    e_yaw = wrap(float(Xr[2]) - yaw)
    info = {"Xr": Xr, "ur": ur, "e_along": e_along, "e_cross": e_cross, "e_yaw": e_yaw}

    if abs(v_ref) < cfg.rotate_speed:               # 제자리 회전·정지 구간
        v = float(np.clip(cfg.k_along * e_along, -cfg.v_max, cfg.v_max))
        w = float(ur[2]) + cfg.k_heading * e_yaw
        info.update(mode="rotate", L=0.0, target=np.array([Xr[0], Xr[1]]), kappa=0.0, lead_s=0.0)
    else:
        v = v_ref + cfg.k_along * e_along
        v = float(np.clip(v, 0.0, cfg.v_max) if v_ref > 0 else np.clip(v, -cfg.v_max, 0.0))
        L = lookahead_distance(v_ref, cfg)
        target, lead = find_lookahead(ref_at, t, (x, y), L, cfg)
        tx, ty = float(target[0]) - x, float(target[1]) - y
        xl, yl = c * tx + s * ty, -s * tx + c * ty
        L2 = max(xl * xl + yl * yl, cfg.lookahead_min ** 2)  # 목표점이 너무 가까우면 곡률이 튀지 않게
        kappa = 2.0 * yl / L2
        w = v * kappa
        info.update(mode="pursuit", L=L, target=np.asarray(target, float), kappa=kappa, lead_s=lead)
    w = float(np.clip(w, -cfg.w_max, cfg.w_max))
    return v, w, info
