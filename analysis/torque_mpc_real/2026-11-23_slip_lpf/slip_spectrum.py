#!/usr/bin/python3
"""slip_logger λ 에 LPF 를 씌울 차단 주파수 근거 (2026-11-23).
  (1) λ 의 주파수 성분 (Welch PSD): 주행 구간, 바퀴별 평균 빼고. 대역별 분산 몫.
  (2) 큰 슬립 사건 (|λ| > 0.5 이 이어진 구간) 의 길이 분포 — 이보다 짧은 사건은 LPF 가 깎는다.
  (3) 차단 주파수별 지표: 1 차 LPF 를 앞뒤로 (filtfilt, 위상 0 — 오프라인) 씌운 뒤 max_i|λ̃_i| > 0.25 / 0.5 시간 %.
"""
import sys
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import welch, butter, filtfilt
st = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
for b in sys.argv[1:]:
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=b, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    T = ["/slip_logger/slip", "/state_estimator/odom", "/torque_mpc/status", "/pure_pursuit/status"]
    r.set_filter(rosbag2_py.StorageFilter(topics=T)); ty = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    D = {k: [] for k in T}
    while r.has_next():
        n, d, t = r.read_next(); D[n].append((t * 1e-9, deserialize_message(d, ty[n])))
    stt = D["/torque_mpc/status"] or D["/pure_pursuit/status"]
    t0, t1 = stt[0][0], stt[-1][0]
    S = np.array([[t, *m.data[1:5]] for t, m in D["/slip_logger/slip"] if len(m.data) >= 5 and t0 <= t <= t1])
    E = np.array([[st(m.header), np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)] for _, m in D["/state_estimator/odom"]])
    fs = 1 / np.median(np.diff(S[:, 0]))
    sp = np.interp(S[:, 0], E[:, 0], E[:, 1])
    # 가장 긴 주행 구간 하나 (속도 > 0.05) 를 PSD 에
    mov = sp > 0.05
    idx = np.flatnonzero(np.diff(np.r_[0, mov.astype(int), 0])); a, e = max(zip(idx[::2], idx[1::2]), key=lambda x: x[1] - x[0])
    L = S[a:e, 1:5]; L = np.where(np.isfinite(L), L, 0.0)
    F, P = welch(L - L.mean(0), fs=fs, nperseg=min(256, len(L)), axis=0); P = P.mean(1)
    tot = np.trapz(P, F)
    band = lambda lo, hi: 100 * np.trapz(P[(F >= lo) & (F < hi)], F[(F >= lo) & (F < hi)]) / tot
    print(f"== {b}: slip_logger {fs:.0f} Hz, 주행 구간 {len(L)/fs:.1f} s")
    print(f"   λ 분산의 주파수 몫: 0–1 Hz {band(0,1):.0f} %, 1–2 {band(1,2):.0f} %, 2–5 {band(2,5):.0f} %, 5–10 {band(5,10):.0f} %, 10–20 {band(10,20):.0f} %, 20–{fs/2:.0f} Hz {band(20,fs/2+1):.0f} %")
    hi = P[F >= 20].mean(); print(f"   PSD 바닥 (20 Hz 위 평균) {hi:.2e} /Hz — 1 Hz {np.interp(1,F,P):.2e}, 2 Hz {np.interp(2,F,P):.2e}, 5 Hz {np.interp(5,F,P):.2e}, 10 Hz {np.interp(10,F,P):.2e}")
    # 큰 슬립 사건 길이 (원신호, 아무 바퀴)
    Lall = np.where(np.isfinite(S[:, 1:5]), S[:, 1:5], 0.0); big = (np.abs(Lall).max(1) > 0.5) & mov
    idx = np.flatnonzero(np.diff(np.r_[0, big.astype(int), 0])); dur = (idx[1::2] - idx[::2]) / fs * 1e3
    if len(dur):
        print(f"   큰 슬립 사건 {len(dur)} 개, 길이 ms: 중앙값 {np.median(dur):.0f}, 사분위 {np.percentile(dur,25):.0f}~{np.percentile(dur,75):.0f}, 90 % {np.percentile(dur,90):.0f}, 최대 {dur.max():.0f}; 10 ms 한 표본짜리 {100*np.mean(dur<=1e3/fs+1):.0f} %")
    print("   차단 주파수 → max|λ̃| > 0.25 % / > 0.5 % (주행 구간, filtfilt 1 차)")
    row = []
    for fc in (None, 20, 10, 5, 2, 1):
        if fc is None:
            Lf = Lall
        else:
            bb, aa = butter(1, fc / (fs / 2)); Lf = filtfilt(bb, aa, Lall, axis=0)
        mx = np.abs(Lf[mov]).max(1)
        row.append(f"{'없음' if fc is None else f'{fc} Hz'}: {100*np.mean(mx>0.25):.1f} / {100*np.mean(mx>0.5):.1f}")
    print("     " + " | ".join(row))
