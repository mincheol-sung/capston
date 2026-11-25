# P1 실차 A·C 궤적 (추정기 자세 = RViz 궤적) 을 기준 경로 위에, 0.5 s 마다 몸체 헤딩 화살표. 경로 좌표 (출발 = 원점, 출발 방향 = +x)
import sys, numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
fm.fontManager.addfont("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
plt.rcParams["font.family"] = fm.FontProperties(fname="/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc").get_name(); plt.rcParams["axes.unicode_minus"] = False
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
out = sys.argv[1]; groups = {"A (PP)": sys.argv[2].split(","), "C (MPC: 엔코더 속도 + 외란 추정 + 슬립 가드)": sys.argv[3].split(",")}
fig, ax = plt.subplots(1, 2, figsize=(13, 7.5))
for col, (name, bags) in enumerate(groups.items()):
    A = ax[col]
    for i, b in enumerate(bags):
        raw = read("bags/2026-11-22/"+b, ("/state_estimator/odom","/figure8/path","/mpc/armed","/dra_mppi/reference_trajectory"))
        s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
        R = np.array([[t, np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
        mv = R[:,1] > 0.05; t0, t1 = R[mv,0][0], R[mv,0][-1]
        path = np.array([[p.pose.position.x,p.pose.position.y] for p in raw["/figure8/path"][-1][1].poses])
        O = np.array([[t, m.pose.pose.position.x, m.pose.pose.position.y, 2*np.arctan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)] for t,m in raw["/state_estimator/odom"] if t0<=t<=t1])
        c0 = path[0]; d = path[5]-path[0]; a0 = np.arctan2(d[1], d[0]); Rm = np.array([[np.cos(-a0),-np.sin(-a0)],[np.sin(-a0),np.cos(-a0)]])
        Pq = (path-c0)@Rm.T; Oq = (O[:,1:3]-c0)@Rm.T; th = O[:,3]-a0
        if i == 0: A.plot(Pq[:,0], Pq[:,1], "k-", lw=1, label="기준 경로")
        clr = ["C0","C1","C2"][i]
        A.plot(Oq[:,0], Oq[:,1], color=clr, lw=1.2, label=b)
    A.set_aspect("equal"); A.grid(alpha=.3); A.legend(fontsize=8, loc="upper right"); A.set_title(name)
    A.set_xlabel("출발 방향 [m]"); A.set_ylabel("왼쪽 [m]")
fig.suptitle("P1 실차 궤적 (추정기 자세 = RViz 궤적), 기준 경로 = 검정")
fig.tight_layout(); fig.savefig(out, dpi=95); print(out)
