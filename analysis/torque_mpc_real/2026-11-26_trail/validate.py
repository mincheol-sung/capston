import sys, math, numpy as np, rosbag2_py
sys.path.insert(0,'/home/smoc/capston/capston/src/amr_basic_stack/controller/mpc')
from amr_mpc.core.trail_fusion import SmoothTrail, slam_to_base, mecanum_fwd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.signal import butter, filtfilt
uri=sys.argv[1]; tau=float(sys.argv[2]) if len(sys.argv)>2 else 1.0
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=uri,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
ty={t.name:t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/state_estimator/odom','/tracked_pose','/wheel_states','/mpc/armed','/imu/base_link']))
st=lambda h:h.stamp.sec+h.stamp.nanosec*1e-9
yaw=lambda q:math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
ub=(sys.argv[4]!="nob") if len(sys.argv)>4 else True
st_=SmoothTrail(mecanum_fwd(0.04,0.1724,0.2140),(0.1093,0.017),tau=tau,use_bias=ub)
E=[];S=[];M=[];I=[];A=[];W=[]
while r.has_next():   # 수신 순서대로 넣는다 (노드와 같은 조건 — SLAM 은 ~110 ms 늦게 도착)
    t,d,ts=r.read_next(); m=deserialize_message(d,get_message(ty[t]))
    if t=='/imu/base_link': st_.on_gyro(m.angular_velocity.z); I.append((st(m.header),m.linear_acceleration.y))
    elif t=='/wheel_states' and len(m.velocity)==4:
        p=st_.on_wheels(st(m.header),m.velocity); W.append((st(m.header),*m.velocity))
        if p: M.append((st(m.header),*p))
    elif t=='/tracked_pose':
        q=m.pose; st_.on_slam(st(m.header),q.position.x,q.position.y,yaw(q.orientation))
        S.append((st(m.header),*slam_to_base(q.position.x,q.position.y,yaw(q.orientation),0.1093,0.017)))
    elif t=='/state_estimator/odom': p=m.pose.pose; E.append((st(m.header),p.position.x,p.position.y,yaw(p.orientation)))
    elif t=='/mpc/armed': A.append((ts*1e-9,m.data))
E,S,M,I,A,W=map(np.array,(E,S,M,I,A,W))
i=np.where(A[:,1]>0.5)[0][0]; t0=A[i,0]; off=A[(A[:,0]>t0)&(A[:,1]<0.5),0]; t1=off.min() if len(off) else E[-1,0]
fs=100.0; T=np.arange(t0+1,t1-1,1/fs); psi=np.interp(T,E[:,0],np.unwrap(E[:,3]))
bb,ab=butter(2,[1/(fs/2),5/(fs/2)],'band'); fi=filtfilt(bb,ab,np.interp(T,I[:,0],I[:,1]))
Tf=mecanum_fwd(0.04,0.1724,0.2140); tw=W[:,0]; vb=(Tf@W[:,1:5].T).T
def stats(name,Tp,P):
    m=(Tp>=t0)&(Tp<=t1); Tp,P=Tp[m],P[m]
    # 고주파 횡흔들림
    lat=[]; v=np.gradient(P,Tp,axis=0); sp=np.linalg.norm(v,axis=1)
    for j in range(len(Tp)):
        w=np.abs(Tp-Tp[j])<=0.25; lat.append(((P[j]-P[w].mean(0))*np.array([-v[j,1],v[j,0]])/max(sp[j],1e-6)).sum())
    lat=np.array(lat)[sp>0.1]
    x=filtfilt(*butter(2,8/(fs/2)),np.interp(T,Tp,P[:,0])); y=filtfilt(*butter(2,8/(fs/2)),np.interp(T,Tp,P[:,1]))
    ay=-np.sin(psi)*np.gradient(np.gradient(x,T),T)+np.cos(psi)*np.gradient(np.gradient(y,T),T); f=filtfilt(bb,ab,ay)
    # SLAM(base) 와의 거리 (저주파 정합)
    dx=np.interp(S[:,0],Tp,P[:,0])-S[:,1]; dy=np.interp(S[:,0],Tp,P[:,1])-S[:,2]; ms=(S[:,0]>=t0+1)&(S[:,0]<=t1-1); dd=np.hypot(dx,dy)[ms]
    step=np.linalg.norm(np.diff(P,axis=0),axis=1); dtp=np.diff(Tp); spd=np.linalg.norm(np.stack([np.interp(Tp[1:],tw,vb[:,0]),np.interp(Tp[1:],tw,vb[:,1])],1),axis=1)
    jump=np.maximum(step-spd*dtp,0)
    print(f"{name:22s} 횡흔들림 RMS {np.sqrt(np.mean(lat**2))*100:4.2f} max {np.abs(lat).max()*100:4.2f} cm | 횡가속 1~5Hz RMS {np.sqrt(np.mean(f**2)):4.2f} (IMU {np.sqrt(np.mean(fi**2)):4.2f}) | SLAM 과 거리 RMS {np.sqrt(np.mean(dd**2))*100:4.2f} max {dd.max()*100:4.2f} cm | 속도 밖 스텝 max {jump.max()*1000:4.1f} mm")
stats("estimator",E[:,0],E[:,1:3]); stats("SLAM→base_footprint",S[:,0],S[:,1:3]); stats(f"smooth τ={tau} bias={ub}",M[:,0],M[:,1:3])
np.savez(sys.argv[3] if len(sys.argv)>3 else '/dev/null',E=E,S=S,M=M,t0=t0,t1=t1)
