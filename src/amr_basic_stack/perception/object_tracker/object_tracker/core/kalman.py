#!/usr/bin/env python3
"""CV(constant velocity) Kalman Filter — 평면 위 물체 하나의 상태추정.

    상태  x = [px, py, vx, vy]^T      추적 프레임(odom)
    측정  z = [px, py]^T              camera_lidar_fusion 이 낸 융합 위치

측정 공분산 R 은 여기서 만들지 않는다. camera_lidar_fusion 이 C2 실측 R(d) 로
채워 `DetectedObject.position_covariance` 에 실어 보낸 것을 그대로 받는다.
즉 이 파일이 정하는 잡음은 프로세스 잡음 Q 하나뿐이다.

Q 는 CWNA (continuous white-noise acceleration) 로 만든다. 가속도를 세기
sigma_a 의 white noise 로 두고 dt 구간을 적분한 것 — Bar-Shalom,
*Estimation with Applications to Tracking and Navigation* 6.2.2절.

한 축(state [p, v], G = [0, 1]^T)에서

    F(tau) G = [tau, 1]^T
    Q = integral_0^dt  F(tau) G sigma_a^2 G^T F(tau)^T  d tau
      = sigma_a^2 integral_0^dt [[tau^2, tau], [tau, 1]] d tau
      = sigma_a^2 [[dt^3/3, dt^2/2],
                   [dt^2/2, dt    ]]

x, y 축은 독립이므로 4x4 로 확장할 때 축 사이 교차항은 0 이다. 아래
`process_noise` 가 이 식이고, `__main__` 에서 수치적분으로 검산한다.

sigma_a 의 단위는 m/s^1.5 다 (m/s^2 가 아니다). Q 의 속도 블록이
sigma_a^2 * dt 이고 이것이 (m/s)^2 여야 하므로 sigma_a^2 는 m^2/s^3 이다.
sigma_a 가 정말 m/s^2 인 모델은 DWNA (discrete white-noise acceleration),
구간마다 가속도가 상수인 쪽으로 Q = sigma_a^2 [[T^4/4, T^3/2], [T^3/2, T^2]]
이다 — Bar-Shalom 6.3.2절.

여기서 CWNA 를 쓰는 이유는 검출 주기가 고르지 않기 때문이다. CWNA 는
Q_vv 가 T 에 선형이라 Q(T1) + Q(T2) = Q(T1+T2) 가 성립해서 어떻게 쪼개
predict 하든 같은 결과가 나온다. DWNA 는 T^2 이라 성립하지 않는다.
1/12s 뒤 1/24s 를 1/8s 한 번과 비교하면 (F Q(T1) F^T + Q(T2) vs Q(T1+T2))

                 max|차이|        Q_vv 합       Q_vv 한번에
    CWNA         0.0e+00        1.250000e-01  = 1.250000e-01
    DWNA         6.9e-03        8.680556e-03 != 1.562500e-02

ApproximateTimeSynchronizer 뒤에서는 dt 가 매 프레임 다르므로 이 성질이
필요하다.

sigma_a 를 굳이 가속도 단위로 읽으려면 기준 시간을 하나 고정해야 한다 —
sigma_a = a * sqrt(tau) 로 두면 "가속도 a 가 tau 초 지속되는 것"에 해당하고,
tau = 1s 를 암묵적으로 쓸 때만 숫자가 m/s^2 와 같아진다.

velocity_chi2 가 STATIC/DYNAMIC 판별의 통계량이다. 임의의 속도 문턱을 두지
않고 "추정된 속도가 자기 불확실성으로 설명되는가"를 묻는다 — camera_lidar_fusion
이 association 게이트에 쓰는 것과 같은 논리다.
"""
import math

import numpy as np

# 측정행렬. 위치만 관측한다.
H = np.array([[1.0, 0.0, 0.0, 0.0],
              [0.0, 1.0, 0.0, 0.0]])

I4 = np.eye(4)


def chi2_2dof(p):
    """자유도 2 카이제곱 분포의 상위 분위수. 닫힌 형태다.

    자유도 2 는 지수분포와 같아 CDF 가 F(x) = 1 - exp(-x/2) 이므로

        x_p = -2 ln(1 - p)

    p=0.99 -> 9.21034, p=0.95 -> 5.99146, p=0.90 -> 4.60517.
    표를 찾거나 scipy 를 부를 필요가 없다.
    """
    if not 0.0 < p < 1.0:
        raise ValueError(f'p 는 (0,1) 이어야 한다: {p}')
    return -2.0 * math.log(1.0 - p)


def transition(dt):
    """F(dt). 등속 모델이라 위치가 속도를 dt 만큼 받는다."""
    F = np.eye(4)
    F[0, 2] = dt
    F[1, 3] = dt
    return F


def process_noise(dt, sigma_a):
    """CWNA 프로세스 잡음 Q(dt). sigma_a 단위 m/s^1.5 (모듈 독스트링 참고)."""
    q = sigma_a * sigma_a
    d3 = dt ** 3 / 3.0
    d2 = dt ** 2 / 2.0
    Q = np.zeros((4, 4))
    Q[0, 0] = Q[1, 1] = q * d3      # 위치-위치
    Q[2, 2] = Q[3, 3] = q * dt      # 속도-속도
    Q[0, 2] = Q[2, 0] = q * d2      # x 축 위치-속도 상관
    Q[1, 3] = Q[3, 1] = q * d2      # y 축 위치-속도 상관
    return Q


class KalmanFilter:
    """x = [px, py, vx, vy], P 4x4."""

    def __init__(self, z, R, sigma_v0):
        """첫 관측으로 초기화. 속도는 0, 그 불확실성은 sigma_v0 로 연다.

        sigma_v0 는 "이 물체가 낼 수 있는 속도의 표준편차"다. 너무 작게 잡으면
        움직이는 물체를 정지로 붙들어 두 프레임쯤 뒤처지고, 너무 크게 잡으면
        첫 update 에서 위치 잡음이 속도로 통째로 새어든다.
        """
        self.x = np.array([float(z[0]), float(z[1]), 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[0:2, 0:2] = R
        self.P[2:4, 2:4] = np.eye(2) * sigma_v0 ** 2

    # ------------------------------------------------------------------ #

    def predict(self, dt, sigma_a):
        F = transition(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + process_noise(dt, sigma_a)

    def innovation(self, z, R):
        """(y, S). association 이 update 없이 이것만 필요로 한다."""
        y = np.asarray(z, dtype=float) - H @ self.x
        S = H @ self.P @ H.T + R
        return y, S

    def update(self, z, R):
        """Joseph form 으로 갱신하고 (y, S) 를 돌려준다.

        표준형 P = (I - KH)P 대신 Joseph form 을 쓰는 이유: 이 필터의 P 는
        출력이 아니라 판정 입력이다. velocity_chi2 가 P_vv 의 역행렬을 쓰므로
        반올림으로 대칭성이나 양정치성이 깨지면 동정 판별이 통째로 틀어진다.
        Joseph form 은 K 가 최적이 아니어도 P 를 대칭 양정치로 유지한다.
        """
        y, S = self.innovation(z, R)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        A = I4 - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)   # 부동소수 비대칭 제거
        return y, S

    # ------------------------------------------------------------------ #

    @property
    def position(self):
        return self.x[0:2]

    @property
    def velocity(self):
        return self.x[2:4]

    @property
    def P_pp(self):
        return self.P[0:2, 0:2]

    @property
    def P_pv(self):
        return self.P[0:2, 2:4]

    @property
    def P_vv(self):
        return self.P[2:4, 2:4]

    @property
    def speed(self):
        return float(math.hypot(self.x[2], self.x[3]))


def mahalanobis(y, S):
    """d^2 = y^T S^-1 y. 특이행렬이면 inf."""
    try:
        return float(y @ np.linalg.solve(S, y))
    except np.linalg.LinAlgError:
        return float('inf')


def velocity_chi2(kf):
    """H0: v = 0 의 검정통계량  d^2 = v^T P_vv^-1 v  ~ chi^2_2.

    이 문턱이 고정 속도 문턱보다 나은 지점은 두 가지인데, 크기가 아주 다르다.
    tools/detection_floor.py 로 잰 값이다 (sigma_a=1.0, 12Hz).

      거리에 따른 적응 — 작다.  1m -> 9m 에서 판정 하한이 반경 471->485mm/s
        (+3%), 횡 473->602mm/s (+27%) 밖에 안 움직인다. C2 융합 위치 잡음이
        6.5m 안에서 1mm 이하 ~ 수 mm 라 P_vv 를 R 이 아니라 Q 가 지배하기
        때문이다. "먼 물체라 자동으로 보수적이 된다"고 말할 만한 크기가 아니다.

      coast 중 적응 — 크다.  검출을 못 받는 동안 P 는 매 스텝 Q 만큼 커진다.
        4m 트랙이 1.0s(= max_coast) coast 하면 하한이 501 -> 3076 mm/s 로
        6.1배 오른다. 즉 오래 못 본 트랙을 외삽 잡음만으로 DYNAMIC 이라
        부르지 않는다. 고정 속도 문턱은 이걸 못 한다.

    P 는 실제 오차 공분산이 아니라 "가속도가 sigma_a 세기로 흔들릴 때"의
    공분산이다. sigma_a 가 실제보다 크면 이 검정은 보수적이 되고(진짜 정지
    물체의 오경보율이 0.99 문턱인데도 0%), 작으면 정지 물체가 DYNAMIC 으로
    샌다. 문턱 이름값을 맞추려면 sigma_a 를 NIS 로 실측해야 한다.
    """
    return mahalanobis(kf.velocity, kf.P_vv)


# ---------------------------------------------------------------------- #

def _selfcheck():
    """Q 를 수치적분으로 검산하고, 등속 물체에서 필터가 수렴하는지 본다."""
    print('=== Q(dt) 검산: 해석해 vs 수치적분 ===')
    sigma_a, dt = 1.3, 0.083          # 라이다 12Hz 주기
    Q = process_noise(dt, sigma_a)
    n = 200001
    tau = np.linspace(0.0, dt, n)
    # 한 축의 integrand: sigma_a^2 [[tau^2, tau], [tau, 1]]
    num = sigma_a ** 2 * np.array([
        [np.trapz(tau ** 2, tau), np.trapz(tau, tau)],
        [np.trapz(tau, tau), np.trapz(np.ones_like(tau), tau)]])
    ana = np.array([[Q[0, 0], Q[0, 2]], [Q[2, 0], Q[2, 2]]])
    print(f'  해석해 {ana.flatten()}')
    print(f'  수치해 {num.flatten()}')
    err = np.max(np.abs(ana - num))
    print(f'  최대 오차 {err:.3e}  -> {"OK" if err < 1e-12 else "FAIL"}')

    print()
    print('=== chi2_2dof 닫힌 형태 ===')
    for p, ref in ((0.90, 4.60517), (0.95, 5.99146), (0.99, 9.21034)):
        got = chi2_2dof(p)
        print(f'  p={p:.2f}  {got:.5f}  (표 {ref})  차이 {abs(got-ref):.2e}')

    print()
    print('=== 등속 물체 추종 (v = 1.0 m/s, 12Hz, sigma_z = 3cm) ===')
    rng = np.random.default_rng(0)
    R = np.eye(2) * 0.03 ** 2
    truth = np.array([0.0, 0.0])
    v_true = np.array([1.0, 0.0])
    kf = KalmanFilter(truth + rng.normal(0, 0.03, 2), R, sigma_v0=1.0)
    for k in range(120):
        truth = truth + v_true * dt
        kf.predict(dt, sigma_a)
        kf.update(truth + rng.normal(0, 0.03, 2), R)
        if k in (4, 11, 35, 119):
            print(f'  k={k:3d}  v_hat=({kf.x[2]:+.3f},{kf.x[3]:+.3f})  '
                  f'|v|={kf.speed:.3f}  chi2_v={velocity_chi2(kf):9.2f}')

    print()
    print('=== 정지 물체 (v = 0) ===')
    kf = KalmanFilter(np.zeros(2), R, sigma_v0=1.0)
    for k in range(120):
        kf.predict(dt, sigma_a)
        kf.update(rng.normal(0, 0.03, 2), R)
        if k in (4, 11, 35, 119):
            print(f'  k={k:3d}  |v|={kf.speed:.3f}  chi2_v={velocity_chi2(kf):9.2f}')
    print()
    print(f'  진입 문턱 chi2_2dof(0.99) = {chi2_2dof(0.99):.3f}')


if __name__ == '__main__':
    _selfcheck()
