# 직선 A 가속 스윕: 명령 accel 별 몸체 실제 가속 (IMU 적분) · 바퀴 가속 · 가속 중 λ · 회전 슬립 (자이로 − 바퀴 기구학 헤딩, 출발 0.3 s)
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
r, k = 0.04, 0.1932
rows = []
for b in sys.argv[1:]:
  try:
      raw = read("bags/2026-11-21/"+b, ("/imu/base_link","/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory"))
      acc = float(open(f"bags/2026-11-21/{b}/run_info/args.txt").read().split("accel:=")[1].split()[0])
      s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
      R = np.array([[t,np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
      tg = R[np.argmax(R[:,1]>0.01),0]
      I = np.array([[t,m.linear_acceleration.x,m.angular_velocity.z] for t,m in raw["/imu/base_link"]])
      pre = (I[:,0]>tg-1.2)&(I[:,0]<tg-0.2); ba, bg = I[pre,1].mean(), I[pre,2].mean()
      kI = (I[:,0]>=tg-0.05)&(I[:,0]<=tg+1.2); tI = I[kI,0]; vI = np.concatenate([[0],np.cumsum(0.5*((I[kI,1]-ba)[1:]+(I[kI,1]-ba)[:-1])*np.diff(tI))])
      W = np.array([[t,*m.velocity] for t,m in raw["/wheel_states"]]); kW = (W[:,0]>=tg-0.05)&(W[:,0]<=tg+1.2); tW = W[kW,0]; vW = r*W[kW,1:5].mean(1)
      def slope(t,v):
          i1 = np.argmax(v>0.1); i9 = np.argmax(v>0.9); return (v[i9]-v[i1])/(t[i9]-t[i1]), t[i9]
      aI, t9 = slope(tI,vI); aW, _ = slope(tW,vW)
      vIw = np.interp(tW,tI,vI); m = (vW>0.3)&(tW<=t9)
      lam = ((vW[m]-vIw[m])/vW[m]).mean()
      t3 = tg+0.3; kg = (I[:,0]>=tg)&(I[:,0]<=t3); kw = (W[:,0]>=tg)&(W[:,0]<=t3)
      g = np.degrees(np.trapz(I[kg,2]-bg, I[kg,0])); w = np.degrees(np.trapz(r*(-W[kw,1]+W[kw,2]-W[kw,3]+W[kw,4])/(4*k), W[kw,0]))
      rows.append((acc, b, aI, aW, lam, g-w))
  except StopIteration:
    print(f"{b}: 무장 안 됨 — 건너뜀")
print("명령 accel | bag | 몸체 실제 가속 | 바퀴 가속 | 바퀴 − 몸체 | 가속 중 λ 평균 | 출발 0.3 s 회전 슬립 (자이로 − 바퀴)")
for acc,b,aI,aW,lam,rs in sorted(rows):
    print(f"{acc:.1f} | {b} | {aI:.2f} m/s² | {aW:.2f} | {aW-aI:+.2f} | {lam:.2f} | {rs:+.1f}°")
