#!/usr/bin/env python3
# 속도 상태 관측기 — Wang Ch.4 (Discrete-time MPC with observer), Maciejowski Ch.5.
# ---------------------------------------------------------------
# 왜 필요한가 (2026-09-09, bags/mpc_chain_20260909_151007 분석):
#
#   body/mpc.py 와 wheel/mpc.py 는 Wang Ch.1.2 의 incremental form 을 쓴다:
#       x = [Δx_m ; y],   Δx_m = x_m(k) − x_m(k−1)
#   x_m 의 속도 블록이 v_e = u_r − v (구조 A) / ω_e = ω_r − ω (구조 B) 이므로
#       Δx_m 의 속도 블록 = −(v(k) − v(k−1))
#   즉 **측정속도의 1차 차분**이 상태로 직행한다. 차분은 white noise 의 σ 를 √2 배로
#   키우는 high-pass 이고, 그게 F = 2ΦᵀQ̄F_free·x 를 거쳐 ΔU = −E⁻¹F 로 나온다.
#
#   실측 (2026-09-09 실차, 390~416 s 주행):
#       /odometry/filtered 잡음 1σ   vx 12.0 mm/s   vy 9.6 mm/s   wz 42.2 mrad/s
#       Δu 부호반전                  vx 63.2 %   vy 67.4 %   wz 56.0 %
#       명령 스펙트럼 봉우리          2.48 Hz  ("슈욱 슈욱" 의 정체)
#   같은 MPC 를 폐루프 시뮬에 넣고 원인을 하나씩 켠 결과 — 잡음만이 지배적이다:
#       아무것도 안 켬        부호반전 37.6 %   평균|Δvx| 0.0010
#       tau 미스매치만        37.6 %            0.0014      ← 기여 없음
#       기준열 재앵커만       40.0 %            0.0037      ← 거의 없음
#       실측 잡음만           59.8 %            0.0256      ← 25 배
#       실차 실측             61.8 %            0.0425
#
# 저역통과와 무엇이 다른가 (이게 핵심이다):
#       저역통과   v̂(k) = (1−α) v̂(k−1) + α v_meas(k)
#       관측기     v̂(k) = a v̂(k−1) + b u(k−1−d) + K (v_meas(k) − v̂(k))
#   저역통과는 입력이 v_meas 뿐이라 잡음을 줄이려면 α 를 낮춰야 하고, 그만큼 위상이 늦는다.
#   잡음 저감과 지연이 한 손잡이에 묶여 있다.
#   관측기는 **주 입력이 명령 u** 다. u 는 우리가 만든 값이라 잡음이 0 이고, FOPDT 모델이
#   위상을 공급한다. 측정은 innovation (v_meas − v̂) 으로만 들어오므로 K 로 잡음 저감만
#   따로 정할 수 있다. 두 손잡이가 분리된다.
#
#   ★ 이 구분이 중요한 이유: EKF 의 lag 을 지우려고 ekf.yaml 의 Q[vx,vy] 를 8.0 까지
#     올린 이력이 있다 (ekf.yaml:80-101). 그 결과 Kalman gain 이 0.9966 이 되어 측정을
#     99.7 % 그대로 통과시킨다 — 실측 잡음 저감비 vx 0.883, wz 1.187 (wz 는 오히려 증폭).
#     저역통과를 여기 새로 넣으면 그때 지웠던 지연을 다시 만드는 셈이라 같은 자리를 맴돈다.
#     관측기는 그 순환에서 빠져나오는 유일한 길이다.
#
# 폐루프 시뮬 비교 (원궤적 R=1 m, 0.30 m/s, 초기오차 15 cm, 플랜트 tau 141 ms / 모델 98 ms,
# 실측 잡음. 6 s 이후 통계):
#       v 를 어디서            부호반전   평균|Δvx|   오차평균   오차최대
#       측정 그대로 (기존)      64.4 %     0.0272     0.06 cm   0.18 cm
#       저역통과 fc=2 Hz        54.4 %     0.0098     0.05      0.16
#       관측기 K=0.30           56.3 %     0.0076     0.03      0.09
#       관측기 K=0.15           55.4 %     0.0043     0.02      0.05   ← 채택
#       관측기 K=0.05           54.1 %     0.0021     0.01      0.02
#       순수예측 K=0            33.2 %     0.0011     0.00      0.00
#   관측기는 저역통과를 **떨림과 추종 양쪽에서** 이긴다 (K=0.15 에서 Δu 2.3 배, 오차 3 배).
#   저역통과는 잡음을 지연과 맞바꾸지만 관측기는 맞바꿀 필요가 없기 때문이다.
#
# K 를 왜 0.15 로 잡았나 — 모델오차·외란 robustness (같은 시뮬, 바퀴 slip 으로 실제 DC
# 이득을 떨어뜨리고 관측기는 계속 k_dc=1.0, tau 98 ms 를 믿게 둔 경우, |Δvx| / 오차평균 / 최대):
#       K       slip 100 %              slip 85 %               slip 70 %
#       0.30    0.0076 / 0.03 / 0.09    0.0077 / 0.03 / 0.09    0.0078 / 0.03 / 0.08
#       0.15    0.0043 / 0.02 / 0.05    0.0044 / 0.02 / 0.05    0.0046 / 0.01 / 0.05
#       0.05    0.0021 / 0.01 / 0.02    0.0023 / 0.01 / 0.02    0.0025 / 0.01 / 0.02
#       0       0.0011 / 0.00 / 0.00    0.0013 / 0.00 / 0.00    0.0016 / 0.00 / 0.00
#   30 % slip 에서도 무너지지 않는다 — MPC 의 출력이 C_m = [I 0 0] 즉 **포즈오차**뿐이고
#   포즈는 별도로 측정되므로, 속도 추정이 다소 틀려도 루프는 포즈 되먹임으로 닫힌다.
#   그래서 K 를 더 낮추면 시뮬 숫자는 계속 좋아진다. 그럼에도 0.15 를 고른 이유:
#   K=0 은 속도 추정이 완전 개루프라 바퀴 stall 같은 지속 외란을 영원히 못 배운다.
#   시뮬 플랜트가 정확히 FOPDT 라 그 실패 모드가 안 나타날 뿐이다. 실차에서 확인하기 전에는
#   보정 경로를 살려 둔다. 실차 로그로 검증한 뒤 낮추는 것은 가능하다.
#
# ★ K 는 plant.yaml / plant_wheel.yaml 의 tuning.k_obs 가 유일한 출처다.
#   k_obs: 1.0 을 주면 v̂ = v_meas 가 되어 이 파일을 넣기 전의 동작과 정확히 같아진다
#   (회귀 비교용). 0 이면 순수 개루프 예측.
# ---------------------------------------------------------------
from collections import deque

import numpy as np


class VelocityObserver:
    """FOPDT 속도 관측기.  구조 A(3축 몸체속도)와 구조 B(4바퀴 각속도)가 같이 쓴다.

        v̂(k) = a v̂(k−1) + b u(k−1−d)              예측 (명령이 주 입력, 잡음 0)
        v̂(k) ← v̂(k) + K (v_meas(k) − v̂(k))        보정 (측정은 innovation 으로만)

    a, b 는 body.model.velocity_block(Ts) / wheel.model.wheel_block(Ts) 가 주는 대각행렬
    그대로다 — 관측기가 예측기와 같은 모델을 쓰게 강제하려고 MPC 가 가진 것을 그대로 받는다.
    """

    def __init__(self, a, b, d, K):
        self.a = np.asarray(a, float)
        self.b = np.asarray(b, float)
        self.d = int(d)
        self.K = np.asarray(K, float)          # 스칼라 또는 축별 배열
        self.n = self.a.shape[0]
        self.u_hist = deque(maxlen=self.d + 1)  # [u(k−1−d), …, u(k−1)]
        self.vhat = None

    def reset(self):
        """기준열 끊김·재시작 때 호출. 다음 update 가 측정으로 다시 초기화한다."""
        self.vhat = None
        self.u_hist.clear()

    def push(self, u):
        """이번 스텝에 실제로 낸 명령을 기록한다. MPC.step 의 return 직전에 부른다."""
        self.u_hist.append(np.asarray(u, float).copy())

    def update(self, v_meas):
        """측정속도를 받아 추정속도를 돌려준다. MPC.step 의 맨 앞에서 부른다."""
        v_meas = np.asarray(v_meas, float)
        # 기동 구간: 지연만큼의 명령 이력이 아직 없으면 측정을 그대로 믿는다.
        # (u_hist 가 maxlen=d+1 이므로 가득 차야 u(k−1−d) 가 u_hist[0] 에 온다.)
        if self.vhat is None or len(self.u_hist) < self.d + 1:
            self.vhat = v_meas.copy()
            return self.vhat
        self.vhat = self.a @ self.vhat + self.b @ self.u_hist[0]   # u_hist[0] = u(k−1−d)
        self.vhat = self.vhat + self.K * (v_meas - self.vhat)
        return self.vhat


def _selftest():
    """1) K=1 이면 측정 그대로 (기존 동작과 동일).
    2) 잡음 없는 FOPDT 를 정확히 재현한다.
    3) 잡음이 있으면 K<1 이 잡음을 K 배 가까이 줄인다."""
    Ts, tau, k_dc, d = 0.04, 0.098, 1.0, 1
    a = np.diag([np.exp(-Ts / tau)] * 3)
    b = np.diag([k_dc * (1 - np.exp(-Ts / tau))] * 3)
    rng = np.random.default_rng(0)

    # 1) K=1 은 pass-through
    ob = VelocityObserver(a, b, d, 1.0)
    for k in range(20):
        vm = rng.normal(0, 1, 3)
        out = ob.update(vm)
        assert np.allclose(out, vm), "K=1 인데 측정과 다르다"
        ob.push(rng.normal(0, 1, 3))

    # 2) 잡음 없는 플랜트를 정확히 따라간다 (K 값과 무관해야 한다)
    for K in (0.0, 0.15, 1.0):
        ob = VelocityObserver(a, b, d, K); v = np.zeros(3); hist = [np.zeros(3)]
        worst = 0.0
        for k in range(200):
            u = np.array([0.3 * np.sin(0.05 * k), 0.1, 0.2])
            worst = max(worst, np.abs(ob.update(v) - v).max())
            ob.push(u); hist.append(u)
            v = a @ v + b @ hist[max(0, len(hist) - 1 - d)]
        assert worst < 1e-9, f"K={K}: 무잡음 재현 오차 {worst:.2e}"

    # 3) 잡음 저감
    sig = 0.012
    out = {}
    for K in (1.0, 0.15):
        ob = VelocityObserver(a, b, d, K); v = np.zeros(3); hist = [np.zeros(3)]; err = []
        for k in range(4000):
            u = np.array([0.3, 0.0, 0.0])
            err.append((ob.update(v + rng.normal(0, sig, 3)) - v)[0])
            ob.push(u); hist.append(u)
            v = a @ v + b @ hist[max(0, len(hist) - 1 - d)]
        out[K] = np.std(err[500:])
    assert out[0.15] < 0.35 * out[1.0], f"잡음 저감이 부족하다: {out}"
    print(f"관측기 자기검사 OK — 추정오차 1σ  K=1.0 {out[1.0]*1000:.2f} mm/s"
          f"  ->  K=0.15 {out[0.15]*1000:.2f} mm/s  ({out[1.0]/out[0.15]:.1f} 배 감소)")
    return True


if __name__ == "__main__":
    raise SystemExit(0 if _selftest() else 1)
