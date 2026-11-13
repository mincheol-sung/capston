#!/usr/bin/env python3
"""NIS (normalized innovation squared) 일관성 진단 — sigma_a 를 실측으로 뽑는다.

Bar-Shalom, *Estimation with Applications to Tracking and Navigation* 5.4절.
필터가 일관적이면 매 프레임의

    NIS = y^T S^-1 y,     y = z - H x_pred,   S = H P_pred H^T + R

가 chi^2_2 를 따른다. 정답 궤적(ground truth)이 필요 없다는 것이 요점이다 —
필터가 자기 예측을 얼마나 틀렸는지와 얼마나 틀릴 것이라 주장했는지만 비교한다.

이 시스템에서 NIS 가 sigma_a 를 식별할 수 있는 이유
-----------------------------------------------------
Bar-Shalom 의 tracking index (maneuver index) Lambda = sigma_a dt^1.5 / sigma_z
가 크면 S 를 R 이 아니라 H P H^T 가 지배하고, H P H^T 는 sigma_a^2 에 비례한다.
C2 실측 R 로 12Hz, 4m 에서 잰 값:

    sigma_a   sqrt(HPH^T)   sqrt(R)    HPH^T/R    Lambda
      0.3        5.88mm     0.54mm       118       13.4
      1.0       19.03mm     0.54mm      1241       44.5
      3.0       56.94mm     0.54mm     11105      133.6

즉 S 가 거의 전부 process noise 다. 진짜 sigma_a 가 1.0 인데 필터가 sa_f 를
쓰면 E[NIS] = trace(S_f^-1 S_true) 가 이렇게 움직인다:

    sa_f    0.25    0.5     1.0     2.0     4.0
    E[NIS] 20.34    6.65    2.00    0.55    0.14

거의 (1/sa_f)^2 이다. 감도가 이만큼 크므로 짧은 bag 으로도 식별된다.

게이트 절단을 반드시 보정해야 한다
------------------------------------
association 이 NIS <= gate 인 짝만 받아들이므로 관측 표본은 오른쪽이 잘린다.
chi^2_2 는 평균 2 의 지수분포라 절단 평균이 닫힌 형태로 나온다.

    E[X | X <= g] = (2 - (g+2) e^(-g/2)) / (1 - e^(-g/2))

g = chi^2_2(0.99) = 9.21034 에서 1.906966 이다 (Monte Carlo 4e6 회 1.906284).
**목표값은 2.000 이 아니라 1.907 이다.** 2.000 을 쓰면 4.9% 계통오차가 된다.

절단 평균만으로는 부족하다. 참 분산비 c = S_true/S_filter 에 대해 절단 평균은
c >= 4 부터 포화하지만(3.74 -> 4.21 -> 4.38 -> 4.51) 기각률은 계속 산다
(31.6% -> 60.0% -> 75.0% -> 88.0%). 그래서 둘을 같이 쓰는 MLE 를 쓴다.

절단 지수분포의 MLE (닫힌 형태)
--------------------------------
받아들인 표본 x_i <= g 의 밀도가 (1/(2c)) exp(-x/(2c)), 기각된 n_rej 개의
확률이 exp(-g/(2c)) 이므로

    l(c) = -n_acc ln(2c) - sum(x_i)/(2c) - n_rej g/(2c)
    dl/dc = 0  =>  c_hat = (sum(x_i) + n_rej g) / (2 n_acc)

Monte Carlo 로 c = 0.25 ~ 16 전 구간에서 확인했다. 순진하게 절단 평균만
1.907 로 나누는 방법은 c >= 2 부터 무너진다.

sigma_a 추정과 반복
--------------------
S 가 sigma_a^2 에 비례하므로 sigma_a_est = sigma_a_filter * sqrt(c_hat).
한 번에 맞지는 않는다 — 필터가 틀린 sigma_a 로 돌면 그 자신의 속도 추정
잡음이 innovation 에 섞여 들어오기 때문이다. 축약사상이라 반복하면 수렴한다.
실제 MultiObjectTracker 로 검증한 값 (90 프레임 x 200 구간, burn-in 3):

    참 0.30:  1.000 -> 0.438 -> 0.340 -> 0.312 -> 0.301
    참 1.00:  1.000 -> 0.987 -> 0.985 -> 0.985 -> 0.985
    참 3.00:  1.000 -> 2.703 -> 3.061 -> 2.968 -> 2.986

고정점 잔차 편향 -1.5%. tools/nis_fit.py 가 이 반복을 돌린다.

표본 수
--------
절단 표준편차가 1.7704 (MC 1.7706) 이므로 평균의 표준오차가 1.77/sqrt(N).
c=1 근처에서 sigma_a 배율의 산포를 400회 반복으로 직접 재면

    N =   50  ->  7.2%        N =  300  ->  2.9%
    N =  100  ->  5.1%        N = 1000  ->  1.6%

트랙 하나 12Hz 기준 N=300 이 25초다. 사람 한 명 걷는 bag 30초면 충분하다.
"""
import math

# ---------------------------------------------------------------------- #


def gated_mean(c=1.0, gate=9.21034):
    """E[NIS | NIS <= gate], NIS = c * chi^2_2.

    c 는 참 분산과 필터가 주장하는 분산의 비다. c=1, gate=chi^2_2(0.99) 에서
    1.906966 — 게이트가 있는 한 관측 평균의 목표는 2 가 아니라 이 값이다.
    """
    a = gate / c
    e = math.exp(-a / 2.0)
    if e >= 1.0:                      # gate/c 가 0 에 가까우면 극한이 gate/2
        return gate / 2.0
    return c * (2.0 - (a + 2.0) * e) / (1.0 - e)


def gated_std(c=1.0, gate=9.21034):
    """절단 표준편차. 표본 수 계산에 쓴다. c=1 에서 1.7704."""
    a = gate / c
    e = math.exp(-a / 2.0)
    if e >= 1.0:
        return 0.0
    m2 = c * c * (8.0 - (a * a + 4.0 * a + 8.0) * e) / (1.0 - e)
    m1 = gated_mean(c, gate)
    return math.sqrt(max(m2 - m1 * m1, 0.0))


def reject_rate(c=1.0, gate=9.21034):
    """게이트 밖으로 나갈 확률. c=1 에서 정확히 1-0.99 = 0.01."""
    return math.exp(-gate / (2.0 * c))


def mle_scale(nis_sum, n_accepted, n_rejected, gate=9.21034):
    """절단 지수분포 MLE.  c_hat = (sum(x) + n_rej*gate) / (2*n_acc).

    n_accepted 가 0 이면 정보가 없으므로 None.
    """
    if n_accepted <= 0:
        return None
    return (nis_sum + n_rejected * gate) / (2.0 * n_accepted)


# ---------------------------------------------------------------------- #


class NisAccumulator:
    """트랙 전체의 NIS 표본을 모아 sigma_a 를 역산한다.

    두 종류를 받는다.
      add_accepted(nis)  게이트를 통과해 update 에 쓰인 짝
      add_rejected()     Hungarian 이 고른 짝인데 게이트가 막은 것

    후자를 세지 않으면 표본이 오른쪽에서 잘린 채로 남아 sigma_a 가 계통적으로
    작게 나온다. 그래서 assign() 이 기각된 chi^2 도 돌려주도록 되어 있다.

    burn_in: 트랙이 생긴 뒤 이만큼의 update 는 표본에서 뺀다. 새 트랙의 P_vv 는
    sigma_v0^2 = 2.25 로 열려 있어 정상상태가 아니고, 이 구간을 넣으면 S 가
    과대평가되어 NIS 가 작게 나온다. n_confirm 과 같은 3 을 기본으로 둔다.
    """

    def __init__(self, gate=9.21034, burn_in=3):
        self.gate = float(gate)
        self.burn_in = int(burn_in)
        self.reset()

    def reset(self):
        self.n_accepted = 0
        self.n_rejected = 0
        self.n_burned = 0
        self.nis_sum = 0.0
        self.nis_max = 0.0

    # ------------------------------------------------------------------ #

    def add_accepted(self, nis, hits=None):
        """hits 는 그 트랙의 누적 검출 수. burn_in 이하면 버린다."""
        if hits is not None and hits <= self.burn_in:
            self.n_burned += 1
            return
        self.n_accepted += 1
        self.nis_sum += float(nis)
        self.nis_max = max(self.nis_max, float(nis))

    def add_rejected(self, n=1):
        self.n_rejected += int(n)

    # ------------------------------------------------------------------ #

    @property
    def n_total(self):
        return self.n_accepted + self.n_rejected

    @property
    def mean(self):
        """받아들인 표본의 평균. 목표는 2.0 이 아니라 gated_mean() 이다."""
        if self.n_accepted == 0:
            return float('nan')
        return self.nis_sum / self.n_accepted

    @property
    def reject_fraction(self):
        if self.n_total == 0:
            return float('nan')
        return self.n_rejected / self.n_total

    @property
    def scale(self):
        """c_hat = (참 분산)/(필터가 주장하는 분산). 1.0 이면 일관적."""
        return mle_scale(self.nis_sum, self.n_accepted, self.n_rejected,
                         self.gate)

    def sigma_a_estimate(self, sigma_a_filter):
        """S 가 sigma_a^2 에 비례하므로 sigma_a * sqrt(c_hat).

        한 번에 맞지 않는다. 이 값을 다시 필터에 넣어 3~4회 반복해야 한다
        (모듈 독스트링의 수렴 표 참고).
        """
        c = self.scale
        if c is None:
            return None
        return float(sigma_a_filter) * math.sqrt(c)

    def relative_error(self):
        """sigma_a 배율 추정의 1-sigma 상대오차. 1.77/sqrt(N) 을 전파한다.

        평균의 표준오차가 sd/sqrt(N) 이고 sigma_a ~ sqrt(c) ~ sqrt(mean) 이라
        델타법으로 상대오차가 그 절반이 된다.
        """
        if self.n_accepted < 2:
            return float('nan')
        c = self.scale or 1.0
        se = gated_std(c, self.gate) / math.sqrt(self.n_accepted)
        return 0.5 * se / max(gated_mean(c, self.gate), 1e-12)

    def report(self, sigma_a_filter):
        """한 줄 요약 문자열. 노드가 주기적으로 로그에 찍는다."""
        if self.n_accepted == 0:
            return f'NIS 표본 없음 (기각 {self.n_rejected}, burn {self.n_burned})'
        est = self.sigma_a_estimate(sigma_a_filter)
        return (f'NIS N={self.n_accepted} 평균={self.mean:.3f} '
                f'(목표 {gated_mean(1.0, self.gate):.3f}) '
                f'기각={self.reject_fraction * 100:.1f}% (목표 '
                f'{reject_rate(1.0, self.gate) * 100:.1f}%) '
                f'c={self.scale:.3f} -> sigma_a {sigma_a_filter:.3f} '
                f'-> {est:.3f} (+-{self.relative_error() * 100:.1f}%, '
                f'반복 필요)')


# ---------------------------------------------------------------------- #


class ModeNisAccumulator:
    """IMM mode 별 NIS 를 mode 확률로 가중해 모은다 — sigma_a 와 sigma_a_ct 분리용.

    왜 NisAccumulator 로는 안 되는가
    ---------------------------------
    NisAccumulator 는 moment-matched 된 fused innovation 하나를 본다. 스칼라
    표본열 하나는 척도 파라미터를 **하나만** 식별한다. 그런데 IMM 의 fused
    S 는 sigma_a^2 에 비례하지 않는다 — moment_match 가 CT mode 의 P (세기가
    sigma_a_ct 로 고정) 와 mode 간 벌어짐 outer(delta, delta) 를 함께 넣기
    때문이다. 그래서 sigma_a * sqrt(c) 가 편향된다. 정상분포가 CT 25% 인
    기본 설정(tau_cv=3.0, tau_ct=1.0)에서 실제로 잰 편향:

        참 sigma_a   fused 로 추정   CT 죽이고 추정
          0.3          0.248 (-17.2%)   0.297 (-0.9%)
          1.0          1.048 ( +4.8%)   0.988 (-1.2%)
          3.0          3.206 ( +6.9%)   3.005 (+0.2%)

    부호가 sigma_a 와 sigma_a_ct(=0.7) 의 대소를 따라 뒤집힌다. CT mode 의
    세기가 sigma_a 와 무관하게 고정돼 있다는 것이 원인이다.

    mode-conditioned NIS
    --------------------
    mode j 의 prior 로 만든 residual r_j 와 S_j = H P_j^- H^T + R 로

        x_ij = r_ij^T S_ij^-1 r_ij

    를 만든다. mode j 가 참이고 그 파라미터가 맞으면 x_ij ~ chi^2_2 다.
    posterior mode 확률 mu_ij 가 "mode j 가 실제로 걸려 있을 확률"이므로
    가중 절단 MLE 를 쓴다 (유도는 위 mle_scale 과 같다. n_acc 를 sum mu 로,
    sum(x) 를 sum(mu x) 로 바꾼 것뿐이다):

        c_j = (sum_i mu_ij x_ij + n_rej_j * gate) / (2 * sum_i mu_ij)

    그리고 sigma_j_est = sigma_j_filter * sqrt(c_j). CV mode 가 sigma_a 를,
    CT mode 가 sigma_a_ct 를 준다.

    게이트 절단을 안분한다 — 근사임을 명시한다
    -------------------------------------------
    게이트는 fused innovation 에 걸린다 (association.cost_matrix 가 fused S
    를 쓴다). 기각된 짝은 estimator.update() 에 도달하지 못하므로 mode 별
    NIS 가 아예 없다. 그래서 기각 수를 누적 평균 mode 확률로 안분한다.
    mode 간 기각 성향이 다르면 틀리는 근사다. 고정점 근처(c -> 1, 기각률
    -> 1%)에서 오차가 작다는 것은 tools/nis_fit.py --sim-imm 이 수치로
    검증한다 — 주장이 아니라 측정으로 확인할 것.
    """

    def __init__(self, gate=9.21034, burn_in=3, n_modes=2):
        self.gate = float(gate)
        self.burn_in = int(burn_in)
        self.n_modes = int(n_modes)
        self.reset()

    def reset(self):
        self.weight = [0.0] * self.n_modes      # sum_i mu_ij
        self.nis_sum = [0.0] * self.n_modes     # sum_i mu_ij x_ij
        self.n_accepted = 0
        self.n_rejected = 0
        self.n_burned = 0

    # ------------------------------------------------------------------ #

    def add_accepted(self, mode_nis, mode_probabilities, hits=None):
        """mode 별 NIS 와 그때의 posterior mode 확률을 함께 받는다."""
        if hits is not None and hits <= self.burn_in:
            self.n_burned += 1
            return
        if len(mode_nis) != self.n_modes:
            return
        if not all(math.isfinite(x) for x in mode_nis):
            return                      # 특이 S 는 표본에서 뺀다
        self.n_accepted += 1
        for j, (x, mu) in enumerate(zip(mode_nis, mode_probabilities)):
            self.weight[j] += float(mu)
            self.nis_sum[j] += float(mu) * float(x)

    def add_rejected(self, n=1):
        self.n_rejected += int(n)

    # ------------------------------------------------------------------ #

    @property
    def mean_probability(self):
        """누적 평균 mode 확률. 기각 수 안분과 정상분포 확인에 쓴다."""
        total = sum(self.weight)
        if total <= 0.0:
            return [float('nan')] * self.n_modes
        return [w / total for w in self.weight]

    def scale(self, mode):
        """c_j = (sum mu x + n_rej_j gate) / (2 sum mu). 1.0 이면 일관적."""
        w = self.weight[mode]
        if w <= 0.0:
            return None
        share = self.mean_probability[mode]
        if not math.isfinite(share):
            return None
        rejected = self.n_rejected * share
        return (self.nis_sum[mode] + rejected * self.gate) / (2.0 * w)

    def mean(self, mode):
        """가중 평균 NIS. 목표는 2.0 이 아니라 gated_mean() 이다."""
        w = self.weight[mode]
        if w <= 0.0:
            return float('nan')
        return self.nis_sum[mode] / w

    def sigma_estimate(self, mode, sigma_filter):
        """sigma_j * sqrt(c_j). 한 번에 맞지 않는다 — 반복이 필요하다."""
        c = self.scale(mode)
        if c is None:
            return None
        return float(sigma_filter) * math.sqrt(c)

    def relative_error(self, mode):
        """유효 표본 수 sum(mu) 를 N 으로 써서 1.77/sqrt(N) 을 전파한다."""
        w = self.weight[mode]
        if w < 2.0:
            return float('nan')
        c = self.scale(mode) or 1.0
        se = gated_std(c, self.gate) / math.sqrt(w)
        return 0.5 * se / max(gated_mean(c, self.gate), 1e-12)


# ---------------------------------------------------------------------- #


class SojournAccumulator:
    """mode 체류 시간(mean sojourn) 을 mode 확률 시퀀스에서 센다.

    ImmEstimator 의 TPM 은 transition_matrix(dt, tau_cv, tau_ct) 로 두 개의
    체류시간이 정한다. 그 둘을 bag 에서 되찾는 방법은 두 가지다.

      1) hard-assign: argmax(mu) 의 run length 평균.  간단하지만 **아래로
         편향**된다 — 12Hz 표본에서 짧은 체류는 한 프레임으로 뭉개지고,
         mu 가 애매한 구간에서 argmax 가 덜덜거리면 run 이 잘게 쪼개진다.
      2) soft-count: 인접 두 프레임의 mu 곱으로 기대 전이수를 세고
         tau_j = -dt / ln(Pi_jj) 를 푼다.  Baum-Welch M-step 의 filtered
         버전이다. 여기 쓰는 것이 이쪽이고, 1) 은 비교용으로 같이 낸다.

    둘이 크게 다르면 mu 가 확신을 못 내고 있다는 뜻이고, 그때는 체류시간이
    아니라 sigma_a / sigma_a_ct 를 먼저 봐야 한다.
    """

    def __init__(self, n_modes=2):
        self.n_modes = int(n_modes)
        self.reset()

    def reset(self):
        n = self.n_modes
        self.transition_mass = [[0.0] * n for _ in range(n)]
        self.dt_sum = 0.0
        self.n_pairs = 0
        self.run_lengths = [[] for _ in range(n)]
        # run 은 트랙별로 센다. 두 트랙이 번갈아 들어오는 것을 한 줄로 세면
        # run 이 잘게 쪼개져 hard_sojourn 이 무의미해진다.
        self._runs = {}

    def _flush_run(self, track_id):
        entry = self._runs.pop(track_id, None)
        if entry is not None and entry[1] > 0:
            self.run_lengths[entry[0]].append(entry[1])

    def end_track(self, track_id):
        """트랙이 사라졌다. 진행 중이던 run 을 확정한다."""
        self._flush_run(track_id)

    def add(self, track_id, mode_probabilities, previous_probabilities, dt):
        """한 트랙의 인접 프레임 쌍 하나. dt 는 그 사이 간격 [s]."""
        if dt <= 0.0 or previous_probabilities is None:
            return
        for i, prev in enumerate(previous_probabilities):
            for j, cur in enumerate(mode_probabilities):
                self.transition_mass[i][j] += float(prev) * float(cur)
        self.dt_sum += float(dt)
        self.n_pairs += 1
        hard = max(range(self.n_modes), key=lambda k: mode_probabilities[k])
        entry = self._runs.get(track_id)
        if entry is not None and entry[0] == hard:
            self._runs[track_id] = (hard, entry[1] + 1)
        else:
            self._flush_run(track_id)
            self._runs[track_id] = (hard, 1)

    # ------------------------------------------------------------------ #

    @property
    def mean_dt(self):
        if self.n_pairs == 0:
            return float('nan')
        return self.dt_sum / self.n_pairs

    def soft_sojourn(self, mode):
        """tau_j = -dt / ln(Pi_jj). Pi_jj 는 기대 전이질량의 행 정규화."""
        row = self.transition_mass[mode]
        total = sum(row)
        if total <= 0.0 or self.n_pairs == 0:
            return float('nan')
        stay = row[mode] / total
        if not 0.0 < stay < 1.0:
            return float('inf') if stay >= 1.0 else float('nan')
        return -self.mean_dt / math.log(stay)

    def hard_sojourn(self, mode):
        """argmax(mu) run length 평균 x dt. 아래로 편향된다."""
        runs = list(self.run_lengths[mode])
        runs += [frames for m, frames in self._runs.values()
                 if m == mode and frames > 0]
        if not runs:
            return float('nan')
        return sum(runs) / len(runs) * self.mean_dt


# ---------------------------------------------------------------------- #

def _selfcheck():
    import numpy as np

    g = 9.21034
    print('=== 절단 통계량: 닫힌 형태 vs Monte Carlo (4e6) ===')
    rng = np.random.default_rng(0)
    x = rng.exponential(2.0, 4_000_000)
    acc = x[x <= g]
    print(f'  평균   닫힌형태 {gated_mean(1.0, g):.6f}   MC {acc.mean():.6f}')
    print(f'  표준편차 닫힌형태 {gated_std(1.0, g):.6f}   MC {acc.std():.6f}')
    print(f'  기각률 닫힌형태 {reject_rate(1.0, g):.6f}   '
          f'MC {(x > g).mean():.6f}')
    assert abs(gated_mean(1.0, g) - acc.mean()) < 5e-3
    assert abs(gated_std(1.0, g) - acc.std()) < 5e-3
    print(f'  -> 목표가 2.000 이 아니라 {gated_mean(1.0, g):.3f} 인 이유가 이것이다.')

    print()
    print('=== MLE: 참 c 를 되찾는가 (절단 평균만 쓰는 방법과 비교) ===')
    print(f'{"참 c":>7} {"MLE c":>9} {"MLE sa배율":>11} {"참 sa배율":>10} '
          f'{"절단평균만":>11} {"기각률":>8}')
    for c in (0.25, 1.0, 2.0, 4.0, 9.0, 16.0):
        s = c * rng.exponential(2.0, 400_000)
        a, r = s[s <= g], int((s > g).sum())
        chat = mle_scale(a.sum(), len(a), r, g)
        naive = a.mean() / gated_mean(1.0, g)
        print(f'{c:7.2f} {chat:9.4f} {math.sqrt(chat):11.3f} '
              f'{math.sqrt(c):10.3f} {math.sqrt(naive):11.3f} '
              f'{r / len(s) * 100:7.2f}%')
        assert abs(chat / c - 1.0) < 0.05
    print('  -> 절단평균만 쓰면 c>=2 부터 포화해서 못 쓴다.')

    print()
    print('=== NisAccumulator: burn-in 과 기각 계수 ===')
    acc_ = NisAccumulator(gate=g, burn_in=3)
    for hits in (1, 2, 3):
        acc_.add_accepted(5.0, hits=hits)
    assert acc_.n_accepted == 0 and acc_.n_burned == 3
    print(f'  hits<=3 은 버려짐: n_accepted={acc_.n_accepted} '
          f'n_burned={acc_.n_burned}')
    s = rng.exponential(2.0, 3000)
    for v in s:
        acc_.add_accepted(v, hits=10) if v <= g else acc_.add_rejected()
    print(f'  {acc_.report(sigma_a_filter=1.0)}')
    assert abs(acc_.sigma_a_estimate(1.0) - 1.0) < 0.05

    print()
    print('=== 표본 수 대 정밀도 (닫힌 형태) ===')
    for n in (50, 100, 300, 1000):
        a2 = NisAccumulator(gate=g)
        a2.n_accepted = n
        a2.nis_sum = n * gated_mean(1.0, g)
        print(f'  N={n:5d}  상대오차 {a2.relative_error() * 100:.1f}%  '
              f'(12Hz 트랙 1개로 {n / 12:.0f}s)')

    print()
    print('전부 통과.')


if __name__ == '__main__':
    _selfcheck()
