"""amr_mpc.core.node_main.spin — 종료 신호 처리 (R9 리뷰 2026-11-14)."""
import os
import signal

import pytest

from amr_mpc.core import node_main


@pytest.fixture
def restore_signals():
    saved = {s: signal.getsignal(s) for s in node_main.STOP_SIGNALS}
    yield
    for s, h in saved.items():
        signal.signal(s, h)


def test_signal_that_lands_in_message_conversion_is_a_clean_stop(monkeypatch, restore_signals):
    """신호가 pybind 변환 중에 떨어지면 KeyboardInterrupt 가 RuntimeError 로 바뀌어 나온다 → 정상 종료로 본다."""
    def fake_spin(_node):
        try:
            os.kill(os.getpid(), signal.SIGTERM)
        except KeyboardInterrupt:
            raise RuntimeError("Unable to convert call argument '0' to Python object")
    monkeypatch.setattr(node_main.rclpy, "spin", fake_spin)
    assert node_main.spin(object()) is True
    assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN          # 정리 중 두 번째 신호는 무시


def test_runtime_error_without_signal_is_raised(monkeypatch, restore_signals):
    def fake_spin(_node):
        raise RuntimeError("진짜 오류")
    monkeypatch.setattr(node_main.rclpy, "spin", fake_spin)
    with pytest.raises(RuntimeError, match="진짜 오류"):
        node_main.spin(object())


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM, signal.SIGHUP])
def test_each_stop_signal_ends_spin(monkeypatch, restore_signals, sig):
    def fake_spin(_node):
        os.kill(os.getpid(), sig)
    monkeypatch.setattr(node_main.rclpy, "spin", fake_spin)
    assert node_main.spin(object()) is True
