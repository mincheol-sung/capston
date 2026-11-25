import numpy as np
np.set_printoptions(precision=4, suppress=True, linewidth=140)
# ---- 현재 값 (torque_plant.yaml / torque_mpc.yaml 작업 트리)
r=0.04; wb=0.1724; wd=0.2140; k=0.5*(wb+wd)
m=4.56; Iz=0.06; Jw=8.9e-4; g=9.81
mw=4*Jw/r**2; meff=m+mw; Ieff=Iz+mw*k**2
print(f"k={k:.4f}  m_w={mw:.4f}  m_eff={meff:.4f}  I_eff={Ieff:.5f}")
F=np.array([[1,1,1,1],[-1,1,1,-1],[-k,k,-k,k]])/r       # (T^-1)^T
M=np.diag([meff,meff,Ieff]); G0=np.linalg.inv(M)@F
print("F=(T^-1)^T =\n",F); print("G(0)=M^-1 F =\n",G0)
n=np.array([1,1,-1,-1])/2; print("F n =",F@n, "   F [1,-1,-1,1]/2 =",F@np.array([1,-1,-1,1])/2)
Ts=0.01
A=np.block([[np.eye(3),Ts*np.eye(3)],[np.zeros((3,3)),np.eye(3)]])
B=np.vstack([0.5*Ts**2*G0,Ts*G0]); E=np.vstack([0.5*Ts**2*np.eye(3),Ts*np.eye(3)])
print("B(0) =\n",B)
# ZOH 정확성 검산: expm of augmented
from scipy.linalg import expm
Ac=np.block([[np.zeros((3,3)),np.eye(3)],[np.zeros((3,3)),np.zeros((3,3))]]); Bc=np.vstack([np.zeros((3,4)),G0])
Maug=np.zeros((10,10)); Maug[:6,:6]=Ac; Maug[:6,6:]=Bc; Ed=expm(Maug*Ts)
print("ZOH expm vs 식-8: |A diff|",np.abs(Ed[:6,:6]-A).max()," |B diff|",np.abs(Ed[:6,6:]-B).max(), "  A_c^2 =0?",np.abs(Ac@Ac).max())
# 샘플링 zero: 위치 출력 x, 입력 몸체 가속 a: (Ts^2/2)(z+1)/(z-1)^2
import scipy.signal as sg
num,den=sg.ss2tf(np.array([[1,Ts],[0,1]]),np.array([[0.5*Ts**2],[Ts]]),np.array([[1,0]]),np.array([[0]]))
print("1축 이중적분 ZOH: num",num,"den",den,"zeros",np.roots(num[0][np.abs(num[0])>1e-15]))
# ---- 토크 상한
tlim=1.3; tfm=0.1; cs=0.8
share=np.array([0.2388,0.2392,0.2608,0.2612]); share=share/share.sum()
N=share*m*g
for label,mu in (("mu_hat_wheels yaml",np.array([0.598,0.489,0.547,0.703])),("최종 비교 mu 0.33 하나",np.full(4,0.33)),("mu 0.3, mg/4 (11-16 발표)",None)):
    if mu is None:
        t=r/np.sqrt(2)*0.3*(m*g/4)*cs; print(f"{label}: tau_mu={t:.4f}")
    else:
        print(f"{label}: N={np.round(N,3)}  tau_mu,i={np.round(r/np.sqrt(2)*mu*N*cs,4)}")
print("motor tau_bar =",tlim-tfm, " rate =", (tlim-tfm)/3)
# 바닥 토크 사상 P
P=np.eye(4)-Jw*F.T@np.linalg.inv(M)@F; print("P (ground map) diag",np.round(np.diag(P),4)," 앞 몫 1-mw/meff=",1-mw/meff)
# ---- 각속도 시절 FOPDT
for nm,taus,Ts_ in (("구조A 축(vx,vy,wz)",[0.098,0.106,0.089],0.04),("구조B 바퀴(FL,FR,RL,RR)",[0.101,0.098,0.102,0.093],0.04),("C++100Hz 바퀴 tau1",[0.043,0.042,0.042,0.038],0.01)):
    t=np.array(taus); a=np.exp(-Ts_/t)
    print(f"{nm}: s=-1/tau={np.round(-1/t,2)} rad/s, Ts={Ts_}: a={np.round(a,4)}, z-pole=a")
