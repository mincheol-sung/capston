# 구름 손실 시뮬 요약: 묶음 (조건 × MPC 값) 별 추종 오차·슬립·토크.  플랜트 구름 손실 5.1 mN·m 고정
import glob, json, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
print("| 조건 | MPC 구름 손실 값 mN·m | 판 | 이동 중 오차 RMS mm (최소~최대) | 최대 오차 mm (최소~최대) | 슬립 10 % 넘는 몫 (최소~최대) | 슬립 최대 (최소~최대) | 토크 RMS · 최대 mN·m |")
print("|---|---|---|---|---|---|---|---|")
for grp in ("C", "G"):
    for tag, val in (("r00", 0.0), ("r27", 2.7), ("r51", 5.1), ("r80", 8.0)):
        v = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(H, "runs", f"{grp}_{tag}_*.json")))]
        v = [d for d in v if d.get("fault", "x") == "" and "moving_rms_mm" in d]
        if not v: continue
        a = lambda k, s=1.0: np.array([d[k] for d in v]) * s
        mr, mx, s10, sm, tr, tm = a("moving_rms_mm"), a("max_mm"), a("slip_frac10"), a("slip_max"), a("tau_rms", 1e3), a("tau_max", 1e3)
        print(f"| {grp} | {val:.1f} | {len(v)} | {mr.mean():.1f} ({mr.min():.1f}~{mr.max():.1f}) | {mx.mean():.1f} ({mx.min():.1f}~{mx.max():.1f}) | "
              f"{s10.mean():.2f} ({s10.min():.2f}~{s10.max():.2f}) | {sm.mean():.2f} ({sm.min():.2f}~{sm.max():.2f}) | {tr.mean():.1f} · {tm.mean():.1f} |")
fl = sorted(os.path.basename(f) for f in glob.glob(os.path.join(H, "runs", "**", "*fault*"), recursive=True))
print("fault 로 다시 돌린 판:", fl if fl else "없음")
