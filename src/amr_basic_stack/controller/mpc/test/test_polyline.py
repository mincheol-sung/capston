"""꺾은선 기준 (core/polyline.py, 2026-11-25) — 정사각형 두 개를 이은 8 자 꼴 기본 경로."""
import math

import numpy as np
import pytest

from amr_mpc.core.polyline import Polyline, parse_path
from amr_mpc.core.square import Square, wrap


def test_default_corners_close_and_two_squares():
    pl = Polyline()
    c = np.array(pl.corners)
    assert np.allclose(c, [[0, 0], [0, -1], [1, -1], [1, 1], [2, 1], [2, 0], [0, 0]], atol=1e-12)
    assert pl.close_err < 1e-12 and pl.L == pytest.approx(8.0)
    # 꼭짓점 회전: 첫 정사각형은 왼쪽 (+90°) 두 번, 둘째는 오른쪽 (−90°) 세 번
    assert [round(math.degrees(t)) for t in pl.turns[1:]] == [90, 90, -90, -90, -90]
    # 2 m 직선 두 개가 맞닿은 점 (1, 0) 을 지난다
    for i in (2, 5):
        a, b = c[i], c[i + 1]
        assert np.allclose(0.5 * (a + b), [1.0, 0.0])


def test_parse_merges_forward_and_rejects_bad():
    assert parse_path("R1, F0.5, L2") == [(-0.5 * math.pi, 1.5), (0.5 * math.pi, 2.0)]
    assert parse_path("우1,좌1") == [(-0.5 * math.pi, 1.0), (0.5 * math.pi, 1.0)]
    for bad in ("", "X1", "R", "R-1", "Rabc"):
        with pytest.raises(ValueError):
            parse_path(bad)


@pytest.mark.parametrize("kw", [dict(), dict(corner_mode="stop"), dict(yaw_mode="fixed"), dict(path="F1,L1,L2,R1,R1,R2"),
                                dict(corner_mode="stop", accel=1.5, v_max=1.0), dict(yaw_mode="fixed", corner_mode="stop")])
def test_velocity_matches_position_derivative_and_returns_to_start(kw):
    pl = Polyline(**kw)
    o = (0.3, -0.2, 0.7)
    h, err = 1e-5, 0.0
    for t in np.linspace(0.005, pl.T_total - 0.005, 4000):
        X1, u = pl.at(t, o)
        X2, _ = pl.at(t + h, o)
        Xm, _ = pl.at(t - h, o)
        d = (X2 - Xm) / (2 * h)
        d[2] = wrap(X2[2] - Xm[2]) / (2 * h)
        c, s = math.cos(X1[2]), math.sin(X1[2])
        body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])
        if np.abs(body - u).max() < 1e-2:                    # 꼭짓점 속도 계단 (pass) 을 걸친 표본은 뺀다 — Square 와 같음
            err = max(err, float(np.abs(body - u).max()))
    assert err < 1e-5
    Xe, ue = pl.at(pl.T_total + 1.0, o)
    assert math.hypot(Xe[0] - o[0], Xe[1] - o[1]) < 1e-9 and abs(wrap(Xe[2] - o[2])) < 1e-9
    assert np.abs(ue).max() == 0.0


def test_start_rotation_only_when_tangent_and_first_turn():
    assert Polyline().T_start == pytest.approx(1.0 + 2 * 0.2)          # rot_s 1.0 + dwell 0.2 × 2
    X, u = Polyline().at(0.7)                                           # 출발 회전 중: 제자리, 오른쪽으로
    assert np.allclose(X[:2], 0.0) and u[2] < 0.0 and abs(u[0]) + abs(u[1]) == 0.0
    assert Polyline(yaw_mode="fixed").T_start == 0.0
    assert Polyline(path="F1,L1,L2,R1,R1,R2").T_start == 0.0


def test_stop_corners_have_zero_body_lateral_speed_and_rotate_in_place():
    pl = Polyline(corner_mode="stop")
    for t in np.linspace(0.0, pl.T_total, 3000):
        _, u = pl.at(t)
        assert abs(u[1]) < 1e-12                                         # tangent stop: 옆걸음 없음 (퓨어 퍼슛도 낼 수 있는 꼴)
        assert abs(u[0]) < 1e-12 or abs(u[2]) < 1e-12                    # 병진과 회전이 겹치지 않는다


def test_same_profile_as_square_for_square_like_path():
    """"F1.5,L1.5,L1.5,L1.5" 은 Square(1.5) 한 바퀴와 같은 기준이다 (출발 회전 없음, 같은 속도·헤딩 규칙)."""
    for mode in ("pass", "stop"):
        sq = Square(side=1.5, corner_speed=0.1, accel=0.4, corner_mode=mode, dwell_s=0.2)
        pl = Polyline(path="F1.5,L1.5,L1.5,L1.5", corner_speed=0.1, accel=0.4, corner_mode=mode, dwell_s=0.2)
        assert pl.T_total == pytest.approx(sq.T_total)
        for t in np.linspace(0.0, sq.T_total, 500):
            Xa, ua = sq.at(t)
            Xb, ub = pl.at(t)
            assert np.allclose(Xa, Xb, atol=1e-9) and np.allclose(ua, ub, atol=1e-9)


def test_rejects_bad_settings():
    with pytest.raises(ValueError):
        Polyline(yaw_mode="spin")
    with pytest.raises(ValueError):
        Polyline(corner_mode="round")
    with pytest.raises(ValueError):
        Polyline(yaw_blend=1.5)                                          # 가장 짧은 직선 1 m 보다 김
