# 롤러 항 값 점검 (2026-11-26): 실차 A/B 6 판에서
#   (1) 외란 추정이 회전 때 배운 회전 가속 (status d̂_ψ / Ts, 몸체 좌표) — 회전 방향 기준 부호. 모델이 맞으면 0, 저항을 모르면 −M_c/I_eff 쪽
#   (2) 회전 중 헤딩 오차 (회전 방향 기준, − = 덜 돎) — 회전 구간 앞 절반 (가속) · 뒤 절반 (감속) 끝에서
#   /usr/bin/python3 param_check.py   (read_bags.py npz 필요)
import glob, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
Ts = 0.01
for grp in ("off", "on"):
    D, E1, E2, EB = [], [], [], []
    for f in sorted(glob.glob(os.path.join(HERE, f"*_rr_{grp}_r*.npz"))):
        A = np.load(f); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]
        R, st, od = A["ref"], A["st"], A["od"]
        yaw = np.unwrap(od[:, 3])
        rot = (R[:, 0] >= t_on) & (np.abs(R[:, 6]) > 0.3)
        idx = np.flatnonzero(rot)
        blocks = np.split(idx, np.flatnonzero(np.diff(idx) > 3) + 1)
        d_rot = []
        for b in blocks:
            if len(b) < 6:
                continue
            sg = np.sign(R[b, 6].mean())
            ms = (st[:, 0] >= R[b[0], 0]) & (st[:, 0] <= R[b[-1], 0])
            d_rot.append(sg * st[ms, 20] / Ts)                      # status 열 1+19 = d̂(5) (회전 속도 증분)
            e = sg * np.degrees(wrap(np.interp(R[b, 0], od[:, 0], yaw) - R[b, 5]))
            E1.append(e[len(b) // 2]); E2.append(e[-1]); EB.append(e.mean())
        D.append(np.concatenate(d_rot).mean())
        print(f"  {os.path.basename(f)[:-4]}: 회전 구간 {len(blocks)} 개, 외란 추정 회전 가속 평균 {D[-1]:+.2f} rad/s²")
    print(f"{grp}: 외란 추정 회전 가속 {np.mean(D):+.2f} rad/s² (판 {np.round(D, 2)}),  헤딩 오차 (+ = 더 돎) 가속 끝 {np.mean(E1):+.2f}° · 감속 끝 {np.mean(E2):+.2f}° · 구간 평균 {np.mean(EB):+.2f}°  (구간 {len(E1)} 개)")
print(f"참고: 저항 0.43 N·m 를 모르면 −0.43/I_eff = {-0.43 / 0.143:+.2f} (I_eff 0.143) ~ {-0.43 / 0.113:+.2f} rad/s² (I_eff 0.113)")
