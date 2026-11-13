#!/usr/bin/env python3
# 회전+병진 동시 상태에서 /odom 과 /odometry/filtered 의 지연·이득을 비교한다.
# ---------------------------------------------------------------
# 왜 이 조건인가: 순수 회전만 재면 두 토픽이 비슷하게 나온다. 8자처럼 회전과 병진이
#   겹칠 때 EKF 쪽만 크게 늦는다. 2026-09-03 에 8자가 발산한 원인이 이거였고,
#   ekf.yaml 의 process_noise_covariance 튜닝도 이 시험으로 잡았다.
#
# 로봇은 (+0.15 m/s, +1.0 rad/s) 와 (-0.15, -1.0) 을 1 초마다 번갈아 낸다 — 되짚는
#   패턴이라 제자리 근처에 남는다. 그래도 주변은 비워라. 약 12 초.
#
# 사용:  python3 tools/combo_lag_test.py
# 판정:  /odometry/filtered 의 vx 지연이 /odom 보다 40 ms 이상 크면 MPC 에 물리지 마라
#        (model.py 의 θ=40 ms 가 그만큼의 여유밖에 없다).
# ---------------------------------------------------------------
import time, math
import numpy as np, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

SEG, RATE = 1.0, 50.0
PLAN = [(0.0,0.0)] + [((0.15,1.0) if i%2==0 else (-0.15,-1.0)) for i in range(10)] + [(0.0,0.0)]

def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

class C(Node):
    def __init__(s):
        super().__init__("combo_test")
        s.t0=s.now(); s.done=False
        s.r={k:[] for k in ("cmd","odom","filt")}
        s.pub=s.create_publisher(Twist,"/cmd_vel",10)
        s.create_subscription(Odometry,"/odom",lambda m:s.add("odom",m),50)
        s.create_subscription(Odometry,"/odometry/filtered",lambda m:s.add("filt",m),50)
        s.create_timer(1/RATE, s.tick)
    def now(s): return s.get_clock().now().nanoseconds*1e-9
    def add(s,k,m):
        w=m.twist.twist; p=m.pose.pose
        s.r[k].append((s.now(), w.linear.x, w.linear.y, w.angular.z, p.position.x, p.position.y, yaw(p.orientation)))
    def tick(s):
        t=s.now()-s.t0; i=int(t//SEG)
        if i>=len(PLAN): s.pub.publish(Twist()); s.done=True; return
        vx,wz=PLAN[i]; tw=Twist(); tw.linear.x=vx; tw.angular.z=wz
        s.pub.publish(tw); s.r["cmd"].append((s.now(),vx,0.0,wz))
    def rep(s):
        s.pub.publish(Twist())
        C_=np.array(s.r["cmd"]); g=np.arange(C_[0,0]+0.5,C_[-1,0]-0.5,0.02)
        print(f"\n{'토픽':>20} {'축':>4} {'이득':>6} {'지연':>7} {'상관':>7}")
        for k,name in (("odom","/odom"),("filt","/odometry/filtered")):
            A=np.array(s.r[k])
            if len(A)<50: print(f"{name:>20}  표본부족"); continue
            for col,ax,ci in ((1,"vx",1),(3,"wz",3)):
                c=np.interp(g,C_[:,0],C_[:,ci])
                best=(0,-2)
                for lag in np.arange(0,0.5,0.01):
                    r=np.corrcoef(c,np.interp(g+lag,A[:,0],A[:,col]))[0,1]
                    if r>best[1]: best=(lag,r)
                o=np.interp(g+best[0],A[:,0],A[:,col])
                print(f"{name:>20} {ax:>4} {np.polyfit(c,o,1)[0]:6.2f} {best[0]*1000:5.0f}ms {best[1]:7.3f}")
        print(f"\n{'토픽':>20} {'총회전°':>9} {'이동 x':>8} {'이동 y':>8}   (되짚는 패턴이라 둘 다 0 근처여야)")
        for k,name in (("odom","/odom"),("filt","/odometry/filtered")):
            A=np.array(s.r[k])
            if len(A)<50: continue
            u=np.unwrap(A[:,6])
            print(f"{name:>20} {math.degrees(u.max()-u.min()):9.1f} {A[-1,4]-A[0,4]:8.3f} {A[-1,5]-A[0,5]:8.3f}")

rclpy.init(); n=C()
try:
    while rclpy.ok() and not n.done: rclpy.spin_once(n,timeout_sec=0.05)
finally:
    for _ in range(10): n.pub.publish(Twist()); time.sleep(0.02)
    n.rep(); n.destroy_node(); rclpy.shutdown()
