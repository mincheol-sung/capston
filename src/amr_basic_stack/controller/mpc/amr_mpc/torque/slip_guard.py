"""바퀴별 슬립 감지와 토크 줄이기 — torque_output_node 가 쓴다 (2026-11-21, ROS 에 의존하지 않는다. 시험 test/test_slip_guard.py).

왜: 토크 MPC (비교군 C) 는 몸체 상태만 보고 바퀴별 속도 되먹임이 없다. 실차 직선 급가속에서 접지가 상한보다 약한 한 바퀴가
    같은 토크를 계속 받아 헛돌거나 (FL, 출발) 잠겼다 (RR, 제동) — analysis/torque_mpc_real/2026-11-21_line_accel/RESULTS_KO.md.
    바퀴별 PI 를 쓰는 비교군 A 는 헛도는 바퀴의 속도 오차가 줄어 그 바퀴 토크가 스스로 줄었다.
    traction control system (TCS) · anti-lock braking (ABS) 와 같은 생각을 바퀴 하나씩에 둔다.

감지 — leave-one-out 기구학 잔차 (이름은 설명용, 정식 용어 아님)
  메카넘 바퀴 속도 ω = T⁻¹·[v_x, v_y, ω_z] (4 식, 미지수 3). ω_z 를 자이로로 알면 미지수 2 · 식 4 라 한 바퀴를 빼고도 남는다.
  바퀴 i 를 뺀 세 식으로 (v_x, v_y) 를 최소제곱으로 맞춰 ω̂_i 를 예측하고 e_i = ω_i − ω̂_i.
  σ_i = e_i·sign(τ_i) / max(|ω̂_i|, floor)  — 양수 = 그 바퀴가 자기 토크 방향으로 바닥보다 더 돈다
  (가속 중 헛돎이면 ω_i > ω̂_i · τ_i > 0, 제동 중 잠김이면 ω_i < ω̂_i · τ_i < 0 — 둘 다 양수).
  ω_z 를 알면 대각 짝 (FL·RR 은 v_x − v_y, FR·RL 은 v_x + v_y) 이 같은 조합을 재서, FL 의 +δ 는 RR 에 −δ 로 똑같이 나온다
  (짝 안에서는 기구학만으로 어느 바퀴인지 모른다). σ = e·sign(τ) 로 토크 방향을 곱해 "자기 토크 방향으로 더 돈 쪽" 을 고른다 —
  구동 중이면 더 빨리 도는 바퀴, 제동 중이면 더 늦게 도는 바퀴. σ 가 가장 큰 바퀴 하나만 후보로 둔다.
  모든 바퀴가 같이 미끄러지는 공통 슬립은 이 잔차에 안 보인다 (기구학이 맞아 떨어진다) — 이 감지기의 한계.

동작 — 바퀴별 이득 g_i ∈ [g_min, 1] 를 순토크에 곱한다
  σ 를 win 표본 이동평균 → 가장 큰 바퀴의 σ 가 threshold 를 min_on 초 넘게 계속 넘으면 그 바퀴를 "슬립" 으로 본다.
  슬립인 바퀴: g_i −= down_rate·dt,  아닌 바퀴: g_i += up_rate·dt  (둘 다 [g_min, 1] 로 자른다).
  bag 으로 미리 본 값 (analysis/torque_mpc_real/2026-11-21_slip_guard/detect_rule.py, 문턱 0.5 · 50 ms):
  C 슬립 판 FL 16~27 % · RR 8~10 % 시간 판정, A 정상 판 0~1.5 %, C 8자·정사각형 FL 5~7 %.
"""
import numpy as np


def loo_residual(omega, wz, Tinv):
    """바퀴마다 나머지 세 바퀴 + 자이로로 예측한 ω̂_i 와 e_i = ω_i − ω̂_i. 반환 (e (4,), ω̂ (4,))."""
    omega = np.asarray(omega, float)
    Tinv = np.asarray(Tinv, float)
    e, pred = np.zeros(4), np.zeros(4)
    for i in range(4):
        j = [k for k in range(4) if k != i]
        y = omega[j] - Tinv[j, 2] * wz
        v = np.linalg.lstsq(Tinv[j, :2], y, rcond=None)[0]
        pred[i] = Tinv[i, :2] @ v + Tinv[i, 2] * wz
        e[i] = omega[i] - pred[i]
    return e, pred


class SlipGuard:
    def __init__(self, Tinv, threshold=0.35, floor=5.0, min_on=0.05, win=3, g_min=0.4, down_rate=8.0, up_rate=2.0):
        if not (threshold > 0.0 and floor > 0.0 and min_on >= 0.0 and win >= 1 and 0.0 < g_min <= 1.0
                and down_rate > 0.0 and up_rate > 0.0):
            raise ValueError("slip guard: threshold·floor·down_rate·up_rate > 0, min_on ≥ 0, win ≥ 1, 0 < g_min ≤ 1")
        self.Tinv = np.asarray(Tinv, float)
        self.th, self.floor, self.min_on, self.win = float(threshold), float(floor), float(min_on), int(win)
        self.g_min, self.down, self.up = float(g_min), float(down_rate), float(up_rate)
        self.reset()

    def reset(self):
        self.g = np.ones(4)
        self.hist = []
        self.cand, self.cand_t = -1, 0.0
        self.flagged = -1
        self.sigma = np.zeros(4)

    def update(self, omega, wz, tau, dt):
        """바퀴 상태 한 번마다. tau = 지금 내고 있는 순토크 (부호만 쓴다). 반환 g (4,)."""
        dt = float(np.clip(dt, 0.0, 0.05))
        e, pred = loo_residual(omega, wz, self.Tinv)
        sig = e * np.sign(np.asarray(tau, float)) / np.maximum(np.abs(pred), self.floor)
        self.hist.append(sig)
        if len(self.hist) > self.win:
            self.hist.pop(0)
        s = np.mean(self.hist, axis=0)
        self.sigma = s
        am = int(np.argmax(s))
        if s[am] > self.th:
            self.cand_t = self.cand_t + dt if am == self.cand else dt
            self.cand = am
        else:
            self.cand, self.cand_t = -1, 0.0
        self.flagged = self.cand if (self.cand >= 0 and self.cand_t >= self.min_on - 1e-9) else -1
        step = np.full(4, self.up * dt)
        if self.flagged >= 0:
            step[self.flagged] = -self.down * dt
        self.g = np.clip(self.g + step, self.g_min, 1.0)
        return self.g.copy()
