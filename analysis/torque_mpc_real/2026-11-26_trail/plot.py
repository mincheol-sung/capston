import sys, numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams['font.family']=['Noto Sans CJK KR','Noto Sans CJK JP','DejaVu Sans']; plt.rcParams['axes.unicode_minus']=False
v=np.load(sys.argv[1]); ref=np.load(sys.argv[2])['ref']; E,S,M,t0,t1=v['E'],v['S'],v['M'],float(v['t0']),float(v['t1'])
k=lambda X:(X[:,0]>=t0)&(X[:,0]<=t1)
fig,ax=plt.subplots(1,2,figsize=(14,7))
for a in ax:
    a.plot(ref[:,0],ref[:,1],color='0.6',lw=1,label='기준 /figure8/path')
    a.plot(E[k(E),1],E[k(E),2],color='tab:green',lw=1,label='estimator (지금 RViz trail)')
    a.plot(S[k(S),1],S[k(S),2],'.-',color='tab:blue',ms=3,lw=0.6,label='SLAM /tracked_pose → base_footprint')
    a.plot(M[k(M),1],M[k(M),2],color='tab:red',lw=1.2,label='smooth (바퀴+자이로, SLAM τ 0.5 s)')
    a.set_aspect('equal'); a.grid(alpha=.3)
ax[0].legend(loc='best',fontsize=8); ax[0].set_title('191922_s2pl_mpc_r1 전체')
# 흔들림이 가장 큰 구간 확대: estimator 스텝 점프 최대 지점
P=E[k(E),1:3]; j=np.argmax(np.linalg.norm(np.diff(P,axis=0),axis=1)); c=P[j]
ax[1].set_xlim(c[0]-0.35,c[0]+0.35); ax[1].set_ylim(c[1]-0.35,c[1]+0.35); ax[1].set_title('estimator 표본 간 이동이 가장 큰 곳 확대 (±35 cm)')
plt.tight_layout(); plt.savefig(sys.argv[3],dpi=130)
