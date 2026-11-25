# analysis/torque_mpc_real/2026-11-21_square/pole.py 와 같은 H·g (torque_mpc.cpp 식-18) — Np·Nc·W 를 인자로, 영공간 방향을 [1,1,-1,-1] 로 고침
import numpy as np
r,k=0.04,0.1932; m_eff,I_eff=6.785,0.14305; Ts=0.01
F=np.array([[1,1,1,1],[-1,1,1,-1],[-k,k,-k,k]])/r
G=np.diag([1/m_eff,1/m_eff,1/I_eff])@F
A=np.block([[np.eye(3),Ts*np.eye(3)],[np.zeros((3,3)),np.eye(3)]]); B=np.vstack([0.5*Ts**2*G,Ts*G])
null=np.array([1,1,-1,-1])/2.0
def gains(W,wd,ru,Np,Nc,qn=0.0):
    nU=4*Nc; Phi=np.zeros((6*Np,nU)); Lam=np.zeros((6*Np,6)); Ai=np.eye(6); Pj=np.zeros((6,nU))
    for i in range(Np):
        c=min(i,Nc-1); Pj=A@Pj; Pj[:,4*c:4*c+4]+=B; Ai=A@Ai; Phi[6*i:6*i+6]=Pj; Lam[6*i:6*i+6]=Ai
    Wb=np.kron(np.eye(Np),np.diag(W)); Hc=np.zeros((nU,nU))
    for j in range(Nc):
        Hc[4*j:4*j+4,4*j:4*j+4]=(wd*(1.0 if j==Nc-1 else 2.0)+ru)*np.eye(4)+qn*np.outer(null,null)*4
        if j+1<Nc:
            Hc[4*j:4*j+4,4*j+4:4*j+8]=-wd*np.eye(4); Hc[4*j+4:4*j+8,4*j:4*j+4]=-wd*np.eye(4)
    H=Phi.T@Wb@Phi+Hc; Hi=np.linalg.inv(H)
    return (Hi@Phi.T@Wb@Lam)[:4], wd*Hi[:4,:4]
def poles(*a,**kw):
    Kx,Kt=gains(*a,**kw)
    Acl=np.block([[A-B@Kx,B@Kt],[-Kx,Kt]]); z,V=np.linalg.eig(Acl); s=np.log(z.astype(complex))/Ts
    out=[]
    for i in range(len(z)):
        v=V[:,i]; e=np.abs(v[:6])**2; px,py,pt=e[0]+e[3],e[1]+e[4],e[2]+e[5]; pn=abs(null@v[6:])**2/ (np.abs(v[6:])**2).sum()*(np.abs(v[6:])**2).sum()
        lab=["x·y","x·y","θ","null"][int(np.argmax([px,py,pt,pn]))]; out.append((lab,s[i],z[i]))
    return out,Kx
def fmt(s):
    wn=abs(s); return (f"{s.real:+.2f}{s.imag:+.2f}j (ζ {-s.real/wn:.2f})" if abs(s.imag)>1e-6 else f"{s.real:+.2f}")
def summary(name,W,wd,ru,Np,Nc,qn=0.0):
    P,Kx=poles(W,wd,ru,Np,Nc,qn)
    res={}
    for ax in ("x·y","θ","null"):
        ps=[p for a,p,_ in P if a==ax]
        if ps: res[ax]=(max(ps,key=lambda p:p.real),max(ps,key=lambda p:abs(p)))
    fa=max((p for _,p,_ in P),key=abs)
    dy=-Kx@np.array([0,0.10,0,0,0,0]); ay=(F@dy)[1]/m_eff
    print(f"| {name} | Np {Np}/Nc {Nc} | W {W} | rΔ {wd:g} | rU {ru:g} | x·y 느린 {fmt(res['x·y'][0])} (τ {-1/res['x·y'][0].real:.2f} s) | θ 느린 {fmt(res['θ'][0])} | 가장 빠른 {fmt(fa)} ({abs(fa)/2/np.pi:.1f} Hz) | null {fmt(res['null'][0]) if 'null' in res else '-'} | 옆 10 cm → 첫 스텝 a_y {ay:+.2f} m/s² |")
print("검증 (11-18(4) 주석: wΔ100 rU1 → 가장 빠른 5.2 Hz ζ0.72; wΔ10 rU1 → 8.9 Hz ζ0.80)")
summary("검증1",[5,5,2.5,2,2,1],100,1,50,5); summary("검증2",[5,5,2.5,2,2,1],10,1,50,5)
print("이력")
summary("1판 시작값 (11-05)",[100,100,50,1,1,0.5],0.01,0.0,50,5,qn=0.01)
summary("11-16 실차",[100,100,50,1,1,0.5],10,0.01,50,5)
summary("11-18(4) 교수님 Q",[5,5,2.5,2,2,1],10,0.1,50,5)
summary("11-20(6) R 키움",[5,5,2.5,2,2,1],100,10,50,5)
summary("11-21(8) Q 확정",[20,20,10,2,2,1],100,10,50,5)
summary("11-23(2) Nc 25",[20,20,10,2,2,1],100,10,50,25)
summary("지금 11-23(4)",[20,20,10,2,2,1],100,10,75,40)
# 지금 설정의 open-loop: A 고유값
print("open-loop A eig:",np.linalg.eigvals(A))
