# 2026-11-26 2 m 직선 두 개에서 헤딩 오차와 옆 밀림 (SLAM 중심 위치, 출발 포즈 기준 좌표) — 속도 출처 바퀴 vs 추정기
#   직선 A: 왼쪽 회전 뒤 x = 1, y −0.8 → 0.8 (헤딩 +90°),  직선 B: 오른쪽 회전 뒤 y = 0, x 1.8 → 0.2 (헤딩 180°)
#   옆 밀림: 진행 방향 왼쪽 + [cm] (직선 A 는 −x, 직선 B 는 −y 가 왼쪽). 헤딩 오차 + = 반시계
import glob, os
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
for grp in ("w", "e"):
    if not glob.glob(os.path.join(HERE, f"*_vs_{grp}_r*.npz")):
        continue
    out = []
    for f in sorted(glob.glob(os.path.join(HERE, f"*_vs_{grp}_r*.npz"))):
        lab = os.path.basename(f)[:-4]
        A = np.load(f); arm = A["arm"]; t_on = arm[arm[:, 1] > 0.5, 0][0]
        od = A["od"]; i0 = np.searchsorted(od[:, 0], t_on); x0, y0, a0 = od[i0, 1], od[i0, 2], od[i0, 3]
        z = np.load(os.path.join(HERE, "slam_cache", lab + ".npz"), allow_pickle=True); T = z["T"]
        c, s = np.cos(a0), np.sin(a0)
        X = c * (T[:, 1] - x0) + s * (T[:, 2] - y0); Y = -s * (T[:, 1] - x0) + c * (T[:, 2] - y0); H = wrap(T[:, 3] - a0)
        a = (np.abs(X - 1.0) < 0.2) & (Y > -0.7) & (Y < 0.8) & (np.abs(wrap(H - np.pi / 2)) < 0.5)
        b = (np.abs(Y) < 0.2) & (X > 0.2) & (X < 1.8) & (np.abs(wrap(H - np.pi)) < 0.5)
        ea, eb = np.degrees(wrap(H[a] - np.pi / 2)).mean(), np.degrees(wrap(H[b] - np.pi)).mean()
        la = -(X[a][np.argmax(Y[a])] - 1.0) * 100           # 직선 A 끝 (y 최대) 에서 왼쪽 (−x) 밀림
        lb = -(Y[b][np.argmin(X[b])]) * 100                 # 직선 B 끝 (x 최소) 에서 왼쪽 (−y) 밀림
        out.append((ea, la, eb, lb))
        print(f"  {lab}: 직선 A 헤딩 오차 {ea:+.2f}° 끝 왼쪽 밀림 {la:+.1f} cm | 직선 B 헤딩 오차 {eb:+.2f}° 끝 왼쪽 밀림 {lb:+.1f} cm")
    o = np.array(out).mean(0)
    print(f"{grp} 평균: 직선 A (왼쪽 회전 뒤) 헤딩 {o[0]:+.2f}° 밀림 {o[1]:+.1f} cm | 직선 B (오른쪽 회전 뒤) 헤딩 {o[2]:+.2f}° 밀림 {o[3]:+.1f} cm")
print(f"참고: 바퀴 구르는 방향이 몸체보다 2.4° 반시계 (11-23) 면 2 m 에서 왼쪽 2·sin(2.4°) = {200 * np.sin(np.radians(2.4)):.1f} cm. 헤딩이 δ° 시계면 2·sin(δ) 만큼 오른쪽")
