"""tools/sensor_lag.py 의 상호상관 지연 (2026-11-19) — 알려진 지연을 되찾고 부호가 '늦으면 +' 인가."""
import importlib.util
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("sensor_lag", os.path.join(HERE, "..", "tools", "sensor_lag.py"))
sl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sl)


def test_recovers_known_delay_and_sign():
    t = np.arange(0, 20, 0.01)
    ref = np.sin(1.3 * t) + 0.5 * np.sin(4.1 * t)
    for d in (0.055, -0.03, 0.0):
        sig = np.sin(1.3 * (t - d)) + 0.5 * np.sin(4.1 * (t - d))      # d 만큼 늦은 신호
        L, r = sl.xcorr_lag(t[50:-50], ref[50:-50], t, sig)
        assert abs(L - d) < 0.006 and r > 0.99


def test_centered_mean_adds_no_delay():
    t = np.arange(0, 20, 0.01)
    x = np.sin(2.0 * t)
    L, r = sl.xcorr_lag(t[50:-50], x[50:-50], t, sl.centered_mean(x, 9))
    assert abs(L) < 0.006
