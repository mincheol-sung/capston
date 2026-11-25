#!/usr/bin/python3
"""C 변 0 시계열 (0.1 s 간격): v_ref, e_t (MPC 가 보는 진행 방향 오차, − 뒤처짐), 모델 가속으로 본 feedforward a_ff = Σ U_s,i sgn(U_s,i)/(m_eff r),
   MPC 출력 a_τ = Σ τ_i sgn(U_s,i)/(m_eff r), a_fb = a_τ − a_ff, 첫 스텝 포화 (max |τ_i|/τ̄_i ≥ 0.98), 활성 제약 수 (status[9], 지평 50 스텝 전체).
   감속 구간은 U_s 부호가 바뀌므로 a_ff·a_τ 는 "제동 크기" 이고 a_fb < 0 = 기준보다 덜 제동.
실행: source /opt/ros/jazzy/setup.bash && source install/setup.bash && python3 analysis/torque_mpc_real/2026-11-21_square/accel_budget.py"""
import importlib.util
import numpy as np
WS="/home/smoc/capston/capston"; MPC=f"{WS}/src/amr_basic_stack/controller/mpc"; WHY=f"{WS}/analysis/torque_mpc_real/2026-11-20_why_c"
spec=importlib.util.spec_from_file_location("cs",f"{MPC}/tools/compare_slam.py"); cs=importlib.util.module_from_spec(spec); spec.loader.exec_module(cs)
spec2=importlib.util.spec_from_file_location("w",f"{WHY}/why_c.py"); w=importlib.util.module_from_spec(spec2); spec2.loader.exec_module(w)
r=0.04; meff=6.785
for bag,tb in [("2026-11-21/183827",np.array([0.1230,0.1253,0.1298,0.1187])),("2026-11-20/190506",np.array([0.1093,0.1113,0.1154,0.1055]))]:
    S,E,P,A=w.load(f"{WS}/bags/{bag}"); D=cs.load(f"{WS}/bags/{bag}")
    t=S[:,0]; tau=S[:,4:8]; us=S[:,22:26]; act=S[:,9]; hold=S[:,20]>0.5
    run=cs.armed_mask(t,A)&~hold&(np.abs(tau).max(1)>1e-6)
    rt=np.array([a for a,_ in D["ref"]])
    ets=[];vr=[];om=[]
    for k,x in enumerate(t):
        R_=D["ref"][max(0,np.searchsorted(rt,x,side="right")-1)][1]; Xr,ur=R_.at(x)[:2]
        th=Xr[2]; c,s=np.cos(th),np.sin(th); vw=np.array([c*ur[0]-s*ur[1],s*ur[0]+c*ur[1]]); n=np.linalg.norm(vw)
        ets.append(-(S[k,1:3]@(vw/max(n,1e-9)))); vr.append(n); om.append(ur[2])
    ets=np.array(ets); vr=np.array(vr); om=np.array(om); i0=np.flatnonzero(run & (vr>0.02))[0]
    sg=np.sign(us); afb=((tau-us)*sg).sum(1)/(meff*r); aff=(us*sg).sum(1)/(meff*r); ato=(tau*sg).sum(1)/(meff*r)
    sat=(np.abs(tau)/tb).max(1)>=0.98
    print("==",bag,"변 0: t, v_ref, e_t(MPC 시각, −뒤처짐) mm, a_ff, a_τ (MPC 출력), a_fb=a_τ−a_ff [m/s²], 포화, 활성 제약 수")
    for tt in np.arange(0.0,2.0,0.1):
        k=i0+np.searchsorted(t[i0:]-t[i0],tt)
        sl=slice(k,k+10)
        print(f"  {tt:3.1f}  v_r {vr[k]:.2f} ω_r {om[k]:+.2f}  e_t {1e3*ets[k]:+5.0f}  a_ff {aff[sl].mean():+.2f}  a_τ {ato[sl].mean():+.2f}  a_fb {afb[sl].mean():+.2f}  포화 {100*sat[sl].mean():3.0f}%  active {act[sl].mean():.1f}")
