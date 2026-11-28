import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
uri=sys.argv[1]
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
ty={t.name:t.type for t in r.get_all_topics_and_types()}
want=['/state_estimator/odom','/tracked_pose','/wheel_states','/mpc/armed','/figure8/path','/imu/base_link']
r.set_filter(rosbag2_py.StorageFilter(topics=want))
E=[];S=[];W=[];A=[];G=[];ref=None
st=lambda h:h.stamp.sec+h.stamp.nanosec*1e-9
yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
while r.has_next():
    t,d,ts=r.read_next(); m=deserialize_message(d,get_message(ty[t]))
    if t=='/state_estimator/odom': p=m.pose.pose; E.append((st(m.header),ts*1e-9,p.position.x,p.position.y,yaw(p.orientation)))
    elif t=='/tracked_pose': p=m.pose; S.append((st(m.header),ts*1e-9,p.position.x,p.position.y,yaw(p.orientation)))
    elif t=='/wheel_states' and len(m.velocity)==4: W.append((st(m.header),*m.velocity))
    elif t=='/imu/base_link': G.append((st(m.header),m.angular_velocity.z))
    elif t=='/mpc/armed': A.append((ts*1e-9,m.data))
    elif t=='/figure8/path' and ref is None: ref=np.array([(q.pose.position.x,q.pose.position.y) for q in m.poses])
E,S,W,G,A=map(np.array,(E,S,W,G,A))
i=np.where(A[:,1]>0.5)[0][0]; t0=A[i,0]; off=A[(A[:,0]>t0)&(A[:,1]<0.5),0]; t1=off.min() if len(off) else E[-1,1]
print(f"armed {t1-t0:.1f}s  E {len(E)}  S {len(S)}  W {len(W)}")
# SLAM 지연 (도착 - stamp)
print(f"SLAM 지연 stamp→수신 중앙 {np.median(S[:,1]-S[:,0])*1e3:.0f} ms, 간격 중앙 {np.median(np.diff(S[:,0]))*1e3:.0f} ms")
k=lambda X,c=0:(X[:,c]>=t0-0.5)&(X[:,c]<=t1+0.5)
E=E[k(E)];S=S[k(S)];W=W[k(W)];G=G[k(G)]
# 1) 경로 기준 횡오차: 각 점에서 기준 경로까지 최단거리
def cte(P):
    d=np.full(len(P),np.inf)
    for a,b in zip(ref[:-1],ref[1:]):
        ab=b-a; L=ab@ab
        if L<1e-12: continue
        u=np.clip(((P-a)@ab)/L,0,1); q=a+u[:,None]*ab; d=np.minimum(d,np.linalg.norm(P-q,axis=1))
    return d
# 2) 고주파 흔들림: 위치를 0.5 s 이동평균에서 뺀 잔차의 진행 직각 성분
def wiggle(T,P,win=0.5):
    out=[]
    for j in range(len(T)):
        m=np.abs(T-T[j])<=win/2; c=P[m].mean(0)
        out.append(P[j]-c)
    R=np.array(out); v=np.gradient(P,T,axis=0); sp=np.linalg.norm(v,axis=1)
    n=np.stack([-v[:,1],v[:,0]],1)/np.maximum(sp,1e-6)[:,None]
    lat=(R*n).sum(1); mv=sp>0.1
    return lat[mv]
# 3) 바퀴 적분 (메카넘 FK, yaw 는 SLAM 처음 값 + IMU 적분)
r_w,k_=0.04,(0.1724+0.2140)/2
Tinv=(1/r_w)*np.array([[1,-1,-k_],[1,1,k_],[1,1,-k_],[1,-1,k_]]); Tf=np.linalg.pinv(Tinv)
tw=W[:,0]; vb=(Tf@W[:,1:5].T).T            # vx vy wz (몸체)
th=np.interp(tw[0],S[:,0],np.unwrap(S[:,4]))+np.concatenate([[0],np.cumsum(np.interp(tw[1:],G[:,0],G[:,1])*np.diff(tw))])
c,s_=np.cos(th),np.sin(th); vw=np.stack([c*vb[:,0]-s_*vb[:,1],s_*vb[:,0]+c*vb[:,1]],1)
Pw=np.concatenate([[S[0,2:4]],S[0,2:4]+np.cumsum(vw[1:]*np.diff(tw)[:,None],0)])
for name,T,P in (("estimator",E[:,0],E[:,2:4]),("SLAM /tracked_pose",S[:,0],S[:,2:4]),("바퀴+IMU 적분",tw,Pw)):
    m=(T>=t0)&(T<=t1); lat=wiggle(T[m],P[m])
    print(f"{name:18s} 0.5s 고주파 횡흔들림 RMS {np.sqrt(np.mean(lat**2))*100:5.2f} cm, max {np.abs(lat).max()*100:5.2f} cm")
# estimator 스텝 점프: 표본 간 이동 - 바퀴 속도로 예측한 이동
m=(E[:,0]>=t0)&(E[:,0]<=t1); Te=E[m,0]; Pe=E[m,2:4]
vwe=np.stack([np.interp(Te,tw,vw[:,0]),np.interp(Te,tw,vw[:,1])],1)
jump=np.linalg.norm(np.diff(Pe,axis=0)-vwe[1:]*np.diff(Te)[:,None],axis=1)
print(f"estimator 표본간 예측 밖 이동: 중앙 {np.median(jump)*1000:.2f} mm, 99% {np.percentile(jump,99)*1000:.1f} mm, max {jump.max()*1000:.1f} mm, >5mm {np.sum(jump>0.005)}회")
# SLAM 표본과 estimator 차이 (같은 stamp)
ex=np.interp(S[:,0],E[:,0],E[:,2]); ey=np.interp(S[:,0],E[:,0],E[:,3]); ms=(S[:,0]>=t0)&(S[:,0]<=t1)
dd=np.hypot(ex-S[:,2],ey-S[:,3])[ms]; print(f"같은 시각 estimator−SLAM 거리: RMS {np.sqrt(np.mean(dd**2))*100:.2f} cm, max {dd.max()*100:.2f} cm")
for name,T,P in (("estimator",E[:,0],E[:,2:4]),("SLAM",S[:,0],S[:,2:4])):
    m=(T>=t0)&(T<=t1); d=cte(P[m]); print(f"{name} 기준경로 횡오차 RMS {np.sqrt(np.mean(d**2))*100:.2f} cm max {d.max()*100:.2f}")
np.savez(sys.argv[2],E=E,S=S,Pw=Pw,tw=tw,ref=ref,t0=t0,t1=t1)
