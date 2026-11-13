"""core/nis.py — 절단 통계량과 MLE. ROS 무관."""
import math

import numpy as np
import pytest

from object_tracker.core.config import TrackerConfig
from object_tracker.core.kalman import chi2_2dof
from object_tracker.core.imm import MODE_COUNT, MODE_CT, MODE_CV
from object_tracker.core.nis import (ModeNisAccumulator, NisAccumulator,
                                     SojournAccumulator, gated_mean,
                                     gated_std, mle_scale, reject_rate)
from object_tracker.core.tracker import MultiObjectTracker

GATE = chi2_2dof(0.99)


# ── 절단 통계량의 닫힌 형태 ──────────────────────────────────────────── #

def test_gated_mean_matches_monte_carlo():
    """게이트가 있으면 목표가 2.0 이 아니라 1.907 이다."""
    rng = np.random.default_rng(0)
    x = rng.exponential(2.0, 2_000_000)      # chi^2_2 == Exp(평균 2)
    assert gated_mean(1.0, GATE) == pytest.approx(x[x <= GATE].mean(), abs=5e-3)
    assert gated_mean(1.0, GATE) == pytest.approx(1.906966, abs=1e-5)


def test_gated_mean_is_below_two():
    """절단은 반드시 평균을 낮춘다. 2.0 을 목표로 쓰면 계통오차가 생긴다."""
    assert gated_mean(1.0, GATE) < 2.0
    # 게이트를 넓히면 2 로 수렴한다.
    assert gated_mean(1.0, 200.0) == pytest.approx(2.0, abs=1e-3)


def test_gated_std_matches_monte_carlo():
    rng = np.random.default_rng(1)
    x = rng.exponential(2.0, 2_000_000)
    assert gated_std(1.0, GATE) == pytest.approx(x[x <= GATE].std(), abs=5e-3)


def test_reject_rate_is_one_minus_p():
    """게이트를 chi^2_2(p) 로 잡았으면 기각률이 정확히 1-p 여야 한다."""
    for p in (0.90, 0.95, 0.99):
        assert reject_rate(1.0, chi2_2dof(p)) == pytest.approx(1.0 - p,
                                                               abs=1e-12)


# ── MLE ──────────────────────────────────────────────────────────────── #

@pytest.mark.parametrize('c', [0.25, 0.5, 1.0, 2.0, 4.0, 9.0, 16.0])
def test_mle_recovers_scale(c):
    """절단이 심해도(c=16 이면 기각률 75%) MLE 는 c 를 되찾는다."""
    rng = np.random.default_rng(int(c * 100))
    x = c * rng.exponential(2.0, 300_000)
    acc, n_rej = x[x <= GATE], int((x > GATE).sum())
    assert mle_scale(acc.sum(), len(acc), n_rej, GATE) == pytest.approx(
        c, rel=0.06)


def test_naive_truncated_mean_fails_where_mle_works():
    """절단 평균만 쓰면 c>=4 에서 포화한다. MLE 를 쓰는 이유."""
    rng = np.random.default_rng(7)
    x = 9.0 * rng.exponential(2.0, 300_000)
    acc, n_rej = x[x <= GATE], int((x > GATE).sum())
    naive = acc.mean() / gated_mean(1.0, GATE)
    assert naive < 3.0                                   # 참 9.0 에 한참 못 미침
    assert mle_scale(acc.sum(), len(acc), n_rej, GATE) == pytest.approx(
        9.0, rel=0.06)


def test_mle_none_without_samples():
    assert mle_scale(0.0, 0, 5, GATE) is None


# ── NisAccumulator ───────────────────────────────────────────────────── #

def test_burn_in_drops_early_updates():
    """새 트랙은 P_vv = sigma_v0^2 라 정상상태가 아니다. 표본에서 뺀다."""
    acc = NisAccumulator(gate=GATE, burn_in=3)
    for hits in (1, 2, 3):
        acc.add_accepted(5.0, hits=hits)
    assert acc.n_accepted == 0 and acc.n_burned == 3
    acc.add_accepted(5.0, hits=4)
    assert acc.n_accepted == 1


def test_accumulator_recovers_consistent_filter():
    acc = NisAccumulator(gate=GATE)
    rng = np.random.default_rng(2)
    for v in rng.exponential(2.0, 20_000):
        acc.add_accepted(v, hits=99) if v <= GATE else acc.add_rejected()
    assert acc.scale == pytest.approx(1.0, rel=0.05)
    assert acc.sigma_a_estimate(1.0) == pytest.approx(1.0, rel=0.05)
    assert acc.reject_fraction == pytest.approx(0.01, abs=0.005)


def test_sigma_a_estimate_scales_linearly():
    """S 가 sigma_a^2 에 비례하므로 추정도 필터값에 비례해야 한다."""
    acc = NisAccumulator(gate=GATE)
    acc.n_accepted, acc.nis_sum = 1000, 1000 * 4.0
    assert acc.sigma_a_estimate(2.0) == pytest.approx(
        2.0 * acc.sigma_a_estimate(1.0))


def test_empty_accumulator_is_safe():
    acc = NisAccumulator(gate=GATE)
    assert acc.scale is None
    assert acc.sigma_a_estimate(1.0) is None
    assert math.isnan(acc.mean)
    assert 'NIS 표본 없음' in acc.report(1.0)


def test_relative_error_shrinks_as_sqrt_n():
    prev = None
    for n in (100, 400, 1600):
        acc = NisAccumulator(gate=GATE)
        acc.n_accepted, acc.nis_sum = n, n * gated_mean(1.0, GATE)
        if prev is not None:
            assert acc.relative_error() == pytest.approx(prev / 2.0, rel=0.05)
        prev = acc.relative_error()


# ── 트래커 배선 ──────────────────────────────────────────────────────── #

def test_tracker_without_accumulator_is_unchanged():
    """진단을 끄면 아무것도 모으지 않고 동작도 같아야 한다."""
    cfg = TrackerConfig()
    R = np.eye(2) * 0.004 ** 2
    a, b = MultiObjectTracker(cfg), MultiObjectTracker(cfg, nis=None)
    for k in range(20):
        z = [np.array([3.0 + 0.1 * k, 0.0])]
        a.step(k / 12.0, z, [R])
        b.step(k / 12.0, z, [R])
    assert [t.id for t in a.tracks] == [t.id for t in b.tracks]
    assert a.tracks[0].kf.x == pytest.approx(b.tracks[0].kf.x)


def test_tracker_feeds_accumulator():
    cfg = TrackerConfig()
    acc = NisAccumulator(gate=cfg.gate_chi2, burn_in=0)
    trk = MultiObjectTracker(cfg, nis=acc)
    R = np.eye(2) * 0.004 ** 2
    rng = np.random.default_rng(0)
    for k in range(60):
        z = [np.array([3.0, 0.0]) + rng.normal(0, 0.004, 2)]
        trk.step(k / 12.0, z, [R])
    # 첫 프레임은 트랙 생성이라 update 가 없다.
    assert acc.n_accepted == 59
    assert acc.scale is not None


def test_track_stores_nis_on_update():
    cfg = TrackerConfig()
    trk = MultiObjectTracker(cfg)
    R = np.eye(2) * 0.004 ** 2
    trk.step(0.0, [np.array([3.0, 0.0])], [R])
    assert math.isnan(trk.tracks[0].nis)          # 생성 시점엔 innovation 없음
    trk.step(1 / 12.0, [np.array([3.0, 0.0])], [R])
    assert trk.tracks[0].nis >= 0.0


def test_rejected_pairs_are_reported():
    """게이트 밖 짝을 세지 않으면 표본이 오른쪽에서 잘린다."""
    from object_tracker.core.association import assign

    chi2 = np.array([[1.0, 50.0], [50.0, 30.0]])
    rejected = []
    matched, _, _ = assign(chi2, GATE, rejected)
    assert matched == {0: 0}
    assert rejected == [30.0]                      # (1,1) 이 Hungarian 의 선택


def test_infinite_costs_are_not_counted_as_rejections():
    """inf 는 애초에 후보가 아니므로 기각 표본에 넣으면 안 된다."""
    from object_tracker.core.association import assign

    rejected = []
    assign(np.full((2, 2), np.inf), GATE, rejected)
    assert rejected == []


# ── mode-conditioned NIS (IMM) ───────────────────────────────────────── #

def test_mode_accumulator_recovers_weighted_scale():
    """가중 MLE 는 mode 확률로 가중한 표본에서 c 를 되찾아야 한다."""
    rng = np.random.default_rng(3)
    acc = ModeNisAccumulator(gate=GATE, burn_in=0)
    true_c = 1.0
    for _ in range(20000):
        x = true_c * rng.exponential(2.0, 2)
        if x[MODE_CV] > GATE:                 # 게이트는 CV 쪽에 건다고 가정
            acc.add_rejected()
            continue
        acc.add_accepted(tuple(x), (0.8, 0.2), hits=10)
    assert acc.scale(MODE_CV) == pytest.approx(true_c, rel=0.05)
    assert acc.mean_probability[MODE_CV] == pytest.approx(0.8, abs=1e-6)


def test_mode_accumulator_honours_burn_in():
    acc = ModeNisAccumulator(gate=GATE, burn_in=3)
    for hits in (1, 2, 3):
        acc.add_accepted((5.0, 5.0), (0.5, 0.5), hits=hits)
    assert acc.n_accepted == 0 and acc.n_burned == 3
    acc.add_accepted((5.0, 5.0), (0.5, 0.5), hits=4)
    assert acc.n_accepted == 1


def test_mode_accumulator_drops_singular_samples():
    """특이 S 는 inf 를 내므로 표본에 들어가면 c 가 오염된다."""
    acc = ModeNisAccumulator(gate=GATE, burn_in=0)
    acc.add_accepted((float('inf'), 1.0), (0.5, 0.5), hits=10)
    assert acc.n_accepted == 0


def test_mode_accumulator_is_safe_when_empty():
    acc = ModeNisAccumulator(gate=GATE)
    assert acc.scale(MODE_CV) is None
    assert acc.sigma_estimate(MODE_CV, 1.0) is None
    assert math.isnan(acc.mean(MODE_CV))
    assert math.isnan(acc.relative_error(MODE_CV))


def test_sigma_estimate_scales_with_sqrt_c():
    acc = ModeNisAccumulator(gate=GATE, burn_in=0)
    for _ in range(1000):
        acc.add_accepted((4.0, 4.0), (1.0, 0.0), hits=10)
    # c = 4/2 = 2 이므로 sigma 배율이 sqrt(2).
    assert acc.scale(MODE_CV) == pytest.approx(2.0)
    assert acc.sigma_estimate(MODE_CV, 1.0) == pytest.approx(math.sqrt(2.0))


def test_tracker_feeds_mode_accumulator_rejections():
    """기각 수가 mode 누적기에도 가야 한다.

    이 배선을 빼먹으면 절단 보정이 통째로 빠져 고정점이 -4.9% 밀린다
    (sigma_a 고정점 0.951 vs 0.984). 눈에 안 띄는 종류의 버그라 테스트로 막는다.
    """
    cfg = TrackerConfig()
    modes = ModeNisAccumulator(gate=cfg.gate_chi2, burn_in=0, n_modes=MODE_COUNT)
    trk = MultiObjectTracker(cfg, mode_nis=modes)
    R = np.eye(2) * 0.004 ** 2
    trk.step(0.0, [np.array([3.0, 0.0])], [R])
    # 게이트를 넘도록 크게 점프시킨다. Hungarian 이 고르지만 게이트가 막는다.
    trk.step(1 / 12.0, [np.array([30.0, 0.0])], [R])
    assert modes.n_rejected == 1


def test_tracker_feeds_mode_accumulator_samples():
    cfg = TrackerConfig()
    modes = ModeNisAccumulator(gate=cfg.gate_chi2, burn_in=0, n_modes=MODE_COUNT)
    trk = MultiObjectTracker(cfg, mode_nis=modes)
    R = np.eye(2) * 0.004 ** 2
    rng = np.random.default_rng(0)
    for k in range(60):
        z = [np.array([3.0, 0.0]) + rng.normal(0, 0.004, 2)]
        trk.step(k / 12.0, z, [R])
    assert modes.n_accepted == 59
    assert modes.weight[MODE_CV] > 0.0 and modes.weight[MODE_CT] > 0.0
    # 두 mode 의 가중치 합이 표본 수여야 한다 (mu 가 정규화돼 있으므로).
    assert sum(modes.weight) == pytest.approx(59.0)


def test_track_stores_mode_nis():
    cfg = TrackerConfig()
    trk = MultiObjectTracker(cfg)
    R = np.eye(2) * 0.004 ** 2
    trk.step(0.0, [np.array([3.0, 0.0])], [R])
    assert all(math.isnan(v) for v in trk.tracks[0].mode_nis)
    trk.step(1 / 12.0, [np.array([3.0, 0.0])], [R])
    assert len(trk.tracks[0].mode_nis) == MODE_COUNT
    assert all(v >= 0.0 for v in trk.tracks[0].mode_nis)
    assert sum(trk.tracks[0].mode_probabilities) == pytest.approx(1.0)


# ── mode 체류 시간 ───────────────────────────────────────────────────── #

def test_soft_sojourn_recovers_known_dwell():
    """mu 가 확신하는 시퀀스에서는 tau = -dt / ln(Pi_jj) 가 맞아야 한다."""
    acc = SojournAccumulator()
    dt = 1.0 / 12.0
    cv, ct = (1.0, 0.0), (0.0, 1.0)
    # CV 에 12 프레임 머물고 CT 로 한 번 넘어가는 것을 100회 반복.
    for _ in range(100):
        previous = cv
        for _ in range(12):
            acc.add(0, cv, previous, dt)
            previous = cv
        acc.add(0, ct, previous, dt)
        acc.add(0, cv, ct, dt)
    # Pi_cv,cv = 1200/1300 이므로 tau = -dt/ln(12/13).
    expected = -dt / math.log(1200.0 / 1300.0)
    assert acc.soft_sojourn(MODE_CV) == pytest.approx(expected, rel=0.02)


def test_hard_sojourn_keeps_tracks_separate():
    """두 트랙이 번갈아 들어와도 run 이 섞이면 안 된다."""
    acc = SojournAccumulator()
    dt = 1.0 / 12.0
    cv, ct = (1.0, 0.0), (0.0, 1.0)
    for _ in range(10):                      # 트랙 0 은 계속 CV, 트랙 1 은 CT
        acc.add(0, cv, cv, dt)
        acc.add(1, ct, ct, dt)
    assert acc.hard_sojourn(MODE_CV) == pytest.approx(10 * dt, rel=1e-6)
    assert acc.hard_sojourn(MODE_CT) == pytest.approx(10 * dt, rel=1e-6)


def test_sojourn_ignores_nonpositive_dt():
    acc = SojournAccumulator()
    acc.add(0, (1.0, 0.0), (1.0, 0.0), 0.0)
    assert acc.n_pairs == 0
    assert math.isnan(acc.soft_sojourn(MODE_CV))


def test_tracker_feeds_sojourn():
    cfg = TrackerConfig()
    sojourn = SojournAccumulator(n_modes=MODE_COUNT)
    trk = MultiObjectTracker(cfg, sojourn=sojourn)
    R = np.eye(2) * 0.004 ** 2
    for k in range(30):
        trk.step(k / 12.0, [np.array([3.0, 0.0])], [R])
    assert sojourn.n_pairs == 29
    assert sojourn.mean_dt == pytest.approx(1 / 12.0, rel=1e-6)
