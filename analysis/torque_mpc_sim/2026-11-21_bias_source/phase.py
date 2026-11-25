#!/usr/bin/python3
"""B 판의 e_c (+ 안쪽) 를 원 안 위상별로: 위상 = 마지막 ω_r 부호 바뀜 뒤 시간 / 그 원의 길이. 전환 뒤 과도인지 원 전체 일정한지 가른다.
   또 기준 ω_r 이 전환에서 계단인지 (기준 생성기가 곡률을 부드럽게 잇는지) 와 전환 폭 [s]."""
import importlib.util, json, sys
import numpy as np
HERE = "/home/smoc/capston/capston/analysis/torque_mpc_sim/2026-11-21_bias_source"; WS = "/home/smoc/capston/capston"
spec = importlib.util.spec_from_file_location("cs", f"{WS}/src/amr_basic_stack/controller/mpc/tools/compare_slam.py")
cs = importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
LAPS = [0.0, 9.0, 15.2, 1e9]
for lab in sys.argv[1:] or ["B4_r1", "B0_r1"]:
    raw = np.load(f"{HERE}/runs/{lab}.npz")["raw"]; t = raw[:, 0]
    et, ec, en, ey, sp = cs.path_errors(*[raw[:, i] for i in range(1, 10)]); om = raw[:, 9]
    mv = sp > 0.02; T0 = t[np.argmax(mv)]; tr = t - T0
    sg = np.sign(om); sw = np.flatnonzero((np.abs(np.diff(sg)) > 0) & (np.abs(om[1:]) > 0.3) | (np.abs(np.diff(sg)) > 0) & (np.abs(om[:-1]) > 0.3))
    # 전환 폭: |ω_r| 가 0.9 |ω|_max 아래로 내려가 있는 시간
    print(f"== {lab}  전환 {len(sw)} 번")
    for k in (1, 2):
        m = mv & (tr >= LAPS[k]) & (tr < LAPS[k + 1])
        idx = np.flatnonzero(m); bins = np.zeros(10); cnt = np.zeros(10)
        segs = [s for s in sw if m[min(s + 1, len(m) - 1)]]
        for a, b in zip(segs[:-1], segs[1:]):
            L = t[b] - t[a]
            ph = ((t[a:b] - t[a]) / L * 10).astype(int).clip(0, 9)
            np.add.at(bins, ph, ec[a:b]); np.add.at(cnt, ph, 1)
        omx = np.abs(om[m]).max(); low = np.mean(np.abs(om[m]) < 0.9 * omx) * (tr[idx[-1]] - tr[idx[0]]) / max(len(segs), 1)
        print(f"   랩 {k+1} (|ω_r| 최대 {omx:.2f}, 전환당 |ω_r|<0.9 최대 인 시간 {low:.2f} s): e_c 위상 0~1 (10 칸) [mm] "
              + " ".join(f"{1e3*b/c:+5.1f}" if c else "  -  " for b, c in zip(bins, cnt)))
