# 마찰 배율 시뮬: 참값 x(τ) 를 기준 출발 시각에 맞춰 묶음 (under / match / over) 평균 → match 와의 차가 어느 구간에서 생기나
#   npz tr = [t, x, y, 기준 이동 중].  τ = t − (기준이 처음 움직인 표본 시각).  기준은 모든 판이 같으므로 x 차 = 진행 방향 추종 오차의 차
import glob, json, os
import numpy as np
H = os.path.dirname(os.path.abspath(__file__))
def load(pat):
    out = []
    for f in sorted(glob.glob(os.path.join(H, "runs", pat + "*.npz"))):
        tr = np.load(f)["tr"]; i0 = np.argmax(tr[:, 3] > 0.5)
        out.append((tr[:, 0] - tr[i0, 0], tr[:, 1], tr[:, 2], tr[:, 3]))
    return out
for grp, T, step in (("G (0.4 m/s, 0.5 m/s²)", 7.6, 0.5), ("C (1.0 m/s, 2.5 m/s², 바닥 μ 0.45)", 4.4, 0.2)):
    g = np.arange(0.0, T, 0.01); X = {}
    for k in ("under", "match", "over"):
        X[k] = np.array([np.interp(g, r[0], r[1]) for r in load(grp[0] + "_" + k)])
    print(f"\n{grp}: 판 수 under {len(X['under'])} · match {len(X['match'])} · over {len(X['over'])}")
    print("| 기준 출발 뒤 s | match 의 x m (속도 m/s) | under − match mm (판 사이 최소~최대) | over − match mm (최소~최대) | match 판 사이 표준편차 mm |")
    print("|---|---|---|---|---|")
    xm = X["match"].mean(0); vm = np.gradient(xm, 0.01)
    for a in np.arange(0.0, T - 1e-9, step):
        s = (g >= a) & (g < a + step)
        du = (X["under"][:, s].mean(1) - xm[s].mean()) * 1e3; do = (X["over"][:, s].mean(1) - xm[s].mean()) * 1e3
        print(f"| {a:.1f} | {xm[s].mean():.3f} ({vm[s].mean():+.2f}) | {du.mean():+.1f} ({du.min():+.1f} ~ {du.max():+.1f}) | {do.mean():+.1f} ({do.min():+.1f} ~ {do.max():+.1f}) | {X['match'][:, s].mean(1).std()*1e3:.1f} |")
print("\n판별 요약 (기준 대비 오차, 기록기 값):")
for k in ("G_under", "G_match", "G_over", "C_under", "C_match", "C_over"):
    v = [json.load(open(f)) for f in sorted(glob.glob(os.path.join(H, "runs", k + "*.json"))) if "fault" not in os.path.basename(f)]
    mr = np.array([d["moving_rms_mm"] for d in v]); mx = np.array([d["max_mm"] for d in v]); tr = np.array([d["tau_rms"] for d in v]) * 1e3
    print(f"  {k:8s} {len(v)} 판  이동 중 RMS {mr.mean():.1f} mm ({mr.min():.1f} ~ {mr.max():.1f})   최대 {mx.mean():.1f} ({mx.min():.1f} ~ {mx.max():.1f})   토크 RMS {tr.mean():.1f} mN·m")
ls = sorted(os.path.basename(f) for f in glob.glob(os.path.join(H, "runs", "*fault*")))
print("fault 로 다시 돌린 판:", ls if ls else "없음")
