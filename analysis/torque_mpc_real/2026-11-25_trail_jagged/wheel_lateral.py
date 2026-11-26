#!/usr/bin/python3
"""바퀴 기구학 옆속도 v_y 를 적분한 옆 변위의 1~5 Hz 몫 — 실제 차체가 옆으로 흔들린 크기 (롤러 미끄럼 포함, 2026-11-25). series.npz 사용."""
import os
import numpy as np
from scipy.signal import butter, filtfilt
H = os.path.dirname(os.path.abspath(__file__)); Z = np.load(os.path.join(H, "series.npz"))
W, E, t0, t1 = Z["W"], Z["E"], float(Z["t0"]), float(Z["t1"])
t = W[:, 0]; om = W[:, 1:5]; r = 0.04
vy_w = r / 4 * (-om[:, 0] + om[:, 1] + om[:, 2] - om[:, 3]); vy_e = np.interp(t, E[:, 0], E[:, 5])
m = (t >= t0) & (t <= t1); dt = np.diff(t, prepend=t[0])
fs = 1 / np.median(np.diff(t)); b, a = butter(2, [1 / (fs / 2), 5 / (fs / 2)], btype="band")
for lab, v in (("바퀴 기구학", vy_w), ("추정기 v_y", vy_e)):
    y = filtfilt(b, a, np.cumsum(v * dt))
    print(f"{lab}: 옆 변위 1~5 Hz 몫 RMS {1e3 * np.sqrt(np.mean(y[m] ** 2)):.1f} mm, 최대 {1e3 * np.abs(y[m]).max():.1f} mm")
