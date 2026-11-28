import sys,math,numpy as np,rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from scipy.signal import butter,filtfilt
z=np.load(sys.argv[2]); E,S,Pw,tw,t0,t1=z['E'],z['S'],z['Pw'],z['tw'],float(z['t0']),float(z['t1'])
r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=sys.argv[1],storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
r.set_filter(rosbag2_py.StorageFilter(topics=['/imu/base_link'])); I=[]
while r.has_next():
    _,d,_=r.read_next(); m=deserialize_message(d,Imu); I.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9,m.linear_acceleration.x,m.linear_acceleration.y))
I=np.array(I); fs=100.0; T=np.arange(t0+1,t1-1,1/fs)
psi=np.interp(T,E[:,0],np.unwrap(E[:,4]))
b,a=butter(2,[1.0/(fs/2),5.0/(fs/2)],'band')
def acc_from_path(Tp,P):
    x=np.interp(T,Tp,P[:,0]); y=np.interp(T,Tp,P[:,1])
    x=filtfilt(*butter(2,8/(fs/2)),x); y=filtfilt(*butter(2,8/(fs/2)),y)
    ax=np.gradient(np.gradient(x,T),T); ay=np.gradient(np.gradient(y,T),T)
    return -np.sin(psi)*ax+np.cos(psi)*ay        # 몸체 y (횡) 가속
src={"IMU 측정":np.interp(T,I[:,0],I[:,2]),
     "estimator 경로 2계미분":acc_from_path(E[:,0],E[:,2:4]),
     "SLAM 경로 2계미분":acc_from_path(S[:,0],S[:,2:4]),
     "바퀴 적분 2계미분":acc_from_path(tw,Pw)}
print("몸체 횡가속 1~5 Hz 성분 RMS [m/s²]  (경로가 정말 그만큼 흔들렸다면 IMU 에 같은 크기로 보여야 함)")
for k,v in src.items():
    f=filtfilt(b,a,v); print(f"  {k:24s} {np.sqrt(np.mean(f**2)):6.2f}   max {np.abs(f).max():6.2f}")
fi=filtfilt(b,a,src["IMU 측정"])
print("IMU 와의 상관 (1~5 Hz) / 경로 성분 중 IMU 로 설명되는 몫")
for k,v in list(src.items())[1:]:
    f=filtfilt(b,a,v); c=np.corrcoef(fi,f)[0,1]
    g=(f@fi)/(fi@fi); res=f-g*fi
    print(f"  {k:24s} 상관 {c:+.2f}   IMU 와 안 맞는 나머지 RMS {np.sqrt(np.mean(res**2)):.2f} m/s²")
