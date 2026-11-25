# 바퀴별 슬립 감지기를 bag 으로 미리 본다 (제어에 안 쓴다).
#   자이로 ω_z (/imu/base_link, 무장 전 1 s 평균 바이어스 뺌) 를 알고, 남은 세 바퀴로 (v_x, v_y) 를 최소제곱으로 맞춘 뒤
#   그 바퀴의 기구학 예측 ω̂_i = T⁻¹[i]·[v_x, v_y, ω_z] 와 실측 ω_i 의 차 e_i  (leave-one-out).
#   σ_i = e_i·sign(τ_i) / max(|ω̂_i|, ω_floor)  — 양수 = 그 바퀴 토크 방향으로 바닥보다 더 돈다 (가속 헛돎 · 제동 잠김 둘 다)
import sys, numpy as np
exec(open("amr/발표/랩미팅/분석_스크립트/track_compare.py").read().split("for b in sys.argv")[0])
Tinv = np.array([[25, -25, -4.83], [25, 25, 4.83], [25, 25, -4.83], [25, -25, 4.83]], float)
FLOOR = 5.0   # rad/s (0.2 m/s)

def loo(omega, wz):
    e, pred = np.zeros(4), np.zeros(4)
    for i in range(4):
        j = [k for k in range(4) if k != i]
        A = Tinv[j, :2]; y = omega[j] - Tinv[j, 2] * wz
        v = np.linalg.lstsq(A, y, rcond=None)[0]
        pred[i] = Tinv[i, :2] @ v + Tinv[i, 2] * wz
        e[i] = omega[i] - pred[i]
    return e, pred

def run(b, top_hint=None):
    raw = read("bags/2026-11-21/"+b, ("/imu/base_link","/wheel_states","/mpc/armed","/dra_mppi/reference_trajectory","/wheel_torque_cmd"))
    s0 = next(t for t,m in raw["/mpc/armed"] if m.data)
    R = np.array([[t,np.hypot(x.vx,x.vy)] for t,m in raw["/dra_mppi/reference_trajectory"] if t>=s0 and len(m.references) for x in [m.references[0]]])
    mv = np.where(R[:,1]>0.01)[0]; tg, te = R[mv[0],0], R[mv[-1],0]
    I = np.array([[t,m.angular_velocity.z] for t,m in raw["/imu/base_link"]]); bg = I[(I[:,0]>tg-1.5)&(I[:,0]<tg-0.6),1].mean()
    W = np.array([[t,*m.velocity] for t,m in raw["/wheel_states"]]); F = np.array([[t,*m.torque] for t,m in raw["/wheel_torque_cmd"]])
    k = (W[:,0]>=tg-0.3)&(W[:,0]<=te+0.5); W = W[k]
    wz = np.interp(W[:,0], I[:,0], I[:,1]-bg)
    tau = np.array([np.interp(W[:,0], F[:,0], F[:,1+i]) for i in range(4)]).T
    E = np.zeros((len(W),4)); Pd = np.zeros((len(W),4))
    for n in range(len(W)): E[n], Pd[n] = loo(W[n,1:5], wz[n])
    sig = E*np.sign(tau)/np.maximum(np.abs(Pd), FLOOR)
    return W[:,0]-tg, sig, E, (te-tg)

if __name__ == "__main__":
    print("bag | 설명 | σ 최대 [FL FR RL RR] | σ>0.3 시간 % [FL FR RL RR] | 처음 σ>0.3 인 바퀴·시각")
    for b, nm in [x.split(":") for x in sys.argv[1:]]:
        t, sig, E, T = run(b)
        over = sig > 0.3
        first = [(t[np.argmax(over[:,i])], i) for i in range(4) if over[:,i].any()]
        f = min(first) if first else None
        print(f"{b} | {nm} | {sig.max(0).round(2)} | {(100*over.mean(0)).round(1)} | " + (f"{'FL FR RL RR'.split()[f[1]]} {f[0]:+.2f} s" if f else "없음"))
