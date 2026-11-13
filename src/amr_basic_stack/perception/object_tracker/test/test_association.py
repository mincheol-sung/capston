"""core.association 단위시험."""
import numpy as np
import pytest

from object_tracker.core.association import assign, cost_matrix
from object_tracker.core.config import TrackerConfig
from object_tracker.core.track import Track


def make_track(track_id, xy, sigma=0.004):
    return Track(track_id, np.array(xy, dtype=float),
                 np.eye(2) * sigma ** 2, 0.0, TrackerConfig())


def test_assign_drops_pairs_outside_gate():
    """Hungarian 이 전역 최적으로 고른 짝도 게이트를 넘으면 버린다."""
    chi2 = np.array([[1.0, 50.0],
                     [50.0, 30.0]])
    matched, unmatched_t, unmatched_m = assign(chi2, gate=9.21)
    assert matched == {0: 0}
    assert unmatched_t == [1]
    assert unmatched_m == [1]


def test_assign_is_globally_optimal_not_greedy():
    """탐욕법이면 (0,1)=0.5 를 먼저 잡아 합 8.5. Hungarian 은 3.0."""
    chi2 = np.array([[2.0, 0.5],
                     [8.0, 1.0]])
    matched, _, _ = assign(chi2, gate=9.21)
    assert matched == {0: 0, 1: 1}


@pytest.mark.parametrize('shape', [(0, 3), (3, 0), (0, 0)])
def test_assign_handles_empty(shape):
    matched, unmatched_t, unmatched_m = assign(np.full(shape, np.inf), gate=9.21)
    assert matched == {}
    assert unmatched_t == list(range(shape[0]))
    assert unmatched_m == list(range(shape[1]))


def test_assign_all_infinite_matches_nothing():
    matched, ut, um = assign(np.full((2, 2), np.inf), gate=9.21)
    assert matched == {} and ut == [0, 1] and um == [0, 1]


def test_cost_matrix_is_smaller_for_the_nearer_measurement():
    trk = make_track(0, [3.0, 0.0])
    R = np.eye(2) * 0.004 ** 2
    chi2 = cost_matrix([trk], [np.array([3.0, 0.0]), np.array([3.5, 0.0])], [R, R])
    assert chi2.shape == (1, 2)
    assert chi2[0, 0] < chi2[0, 1]


def test_cost_matrix_uses_track_covariance():
    """P 가 크게 열린 트랙은 같은 잔차에도 chi^2 가 작다."""
    R = np.eye(2) * 0.004 ** 2
    tight = make_track(0, [0.0, 0.0])
    loose = make_track(1, [0.0, 0.0])
    loose.kf.P *= 100.0
    chi2 = cost_matrix([tight, loose], [np.array([0.1, 0.0])], [R])
    assert chi2[1, 0] < chi2[0, 0]
