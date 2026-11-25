# 직선 경로 y=0 위 unicycle + 퓨어 퍼슛 (κ = 2 y_l / L²), 상태 [e_y, ψ]. 유한차분 야코비안 vs 해석식 s = (v/L)(−1 ± j)
import numpy as np
def f(s,v,L):
    e,psi=s
    dx=np.sqrt(max(L*L-e*e,0.0))               # 룩어헤드 점 = 경로 위 |p − 로봇| = L 인 앞쪽 점 (x_r + dx, 0)
    xl= np.cos(psi)*dx+np.sin(psi)*(-e)          # 몸체 좌표 (R(ψ)ᵀ (p − 로봇))
    yl=-np.sin(psi)*dx+np.cos(psi)*(-e)
    kap=2*yl/(xl*xl+yl*yl)
    return np.array([v*np.sin(psi), v*kap])
for v,L in ((0.7,0.30),(0.7,0.35),(1.0,0.30),(1.0,0.50)):
    h=1e-6; J=np.column_stack([(f(np.array([0,0])+h*np.eye(2)[i],v,L)-f(np.array([0,0])-h*np.eye(2)[i],v,L))/(2*h) for i in range(2)])
    ev=np.linalg.eigvals(J); an=(v/L)*np.array([-1+1j,-1-1j])
    print(f"v {v} L {L}: 유한차분 {np.round(ev,4)}  해석 {np.round(an,4)}  ω_n {abs(ev[0]):.3f}  ζ {-ev[0].real/abs(ev[0]):.3f}")
