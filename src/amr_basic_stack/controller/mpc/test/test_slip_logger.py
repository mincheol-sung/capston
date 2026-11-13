"""slip_logger 의 슬립률 정의 (구조도)."""
import numpy as np
import pytest

from amr_mpc.nodes.slip_logger import slip_ratio


def test_no_slip_is_zero():
    w = np.array([10.0, -5.0, 3.0, 0.0])
    assert np.allclose(slip_ratio(w, w, 2.5), 0.0)


def test_spin_and_skid_signs():
    # 헛돎(바퀴가 더 빠름) = +, 끌림(바퀴가 더 느림) = −, 후진에서도 같은 뜻
    lam = slip_ratio([12.0, 8.0, -12.0, -8.0], [10.0, 10.0, -10.0, -10.0], 2.5)
    assert lam[0] == pytest.approx(2 / 12) and lam[1] == pytest.approx(-2 / 10)
    assert lam[2] == pytest.approx(2 / 12) and lam[3] == pytest.approx(-2 / 10)


def test_bounded_and_no_division_blowup():
    lam = slip_ratio([20.0, 0.0, 0.01, 1.0], [0.0, 20.0, 0.0, 1.2], 2.5)
    assert lam[0] == pytest.approx(1.0)        # 몸체는 서 있는데 바퀴만 돈다: 완전 헛돎
    assert lam[1] == pytest.approx(-1.0)       # 바퀴가 멈춘 채 끌려간다: 완전 잠김
    assert abs(lam[2]) < 0.01                  # 거의 정지한 바퀴는 ω_ε 로 눌린다
    assert np.all(np.abs(lam) <= 1.0 + 1e-12)
    assert slip_ratio([-10.0], [10.0], 2.5)[0] == pytest.approx(-2.0)   # 몸체와 반대로 돈다
