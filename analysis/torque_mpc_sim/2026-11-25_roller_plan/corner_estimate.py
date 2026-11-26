# 롤러 구름 저항 반영 안 — 꺾은선 정지 코너 제자리 90° 회전 (Polyline 기본 rot_s 1.0) 에서 바퀴당 회전 꼴 바닥 토크: 필요한 값 vs 모델이 계획하는 값
#   필요한 값 = (실제 관성 0.115 × 기준 각가속도 + 실제 바닥 회전 저항 0.43 × 회전 방향) / 19.32   (11-25 실차 식별)
#   모델이 계획하는 값 = (모델 관성 × 기준 각가속도 + 모델 저항 × 회전 방향) / 19.32   (띄운 마찰은 양쪽 다 맞게 보상된다고 본다)
#   열린 고리로 굴려 보는 어림은 쓰지 않는다 — 실제 MPC 는 피드백·외란 추정으로 고치므로, 여기서는 "어느 국면에서 얼마나 틀린 계획을 세우나" 만 본다
#   /usr/bin/python3 analysis/torque_mpc_sim/2026-11-25_roller_plan/corner_estimate.py
import math, os, sys
import numpy as np
WS = os.path.realpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(WS, "src/amr_basic_stack/controller/mpc"))
from amr_mpc.core.polyline import Polyline

pl = Polyline(corner_mode="stop")
t = np.arange(0.0, pl.T_total, 1e-3)
wr = np.array([pl.at(tt)[1][2] for tt in t])
on = np.abs(wr) > 1e-6
i0 = np.flatnonzero(on)[0]; i1 = i0 + np.flatnonzero(~on[i0:])[0]
w_ref = np.abs(wr[i0:i1]); tr = t[i0:i1] - t[i0]
a_ref = np.gradient(w_ref, tr)
print(f"기준 (첫 회전, 크기만): {np.degrees(np.sum(w_ref) * 1e-3):.1f}° / {tr[-1]:.2f} s, 최고 회전 속도 {w_ref.max():.2f} rad/s, 각가속도 +{a_ref.max():.2f} ~ {a_ref.min():.2f} rad/s² (3 차 곡선: 양 끝에서 최대)")
C, I_R, M_R = 19.32, 0.115, 0.43
mid_acc = np.argmin(np.abs(tr - 0.25)); mid_dec = np.argmin(np.abs(tr - 0.75))
need = lambda k: (I_R * a_ref[k] + M_R * math.tanh(w_ref[k] / 0.05)) / C * 1e3
print(f"필요한 값 mN·m: 출발 직후 {need(5):+.0f}, 가속 중간 (0.25 s) {need(mid_acc):+.0f}, 감속 중간 (0.75 s) {need(mid_dec):+.0f}, 끝 직전 {need(len(tr) - 6):+.0f}  (상한 약 80)")
print("| 모델 | 가속 중간: 계획 − 필요 | 감속 중간: 계획 − 필요 | 감속 구간 평균 계획 − 필요 |")
for lab, Im, Mm in (("지금 (관성 0.143, 저항 0)", 0.143, 0.0), ("관성만 고침 (0.115, 0)", 0.115, 0.0), ("저항만 넣음 (0.143, 0.43)", 0.143, 0.43),
                    ("둘 다 · 저항 낮게 (0.115, 0.37)", 0.115, 0.37), ("둘 다 (0.115, 0.43)", 0.115, 0.43)):
    plan = lambda k: (Im * a_ref[k] + Mm * math.tanh(w_ref[k] / 0.05)) / C * 1e3
    dec = a_ref < 0
    dd = np.mean([plan(k) - need(k) for k in np.flatnonzero(dec)])
    print(f"| {lab} | {plan(mid_acc) - need(mid_acc):+.1f} | {plan(mid_dec) - need(mid_dec):+.1f} | {dd:+.1f} |")
print("(부호는 회전 방향 기준. − = 회전 방향 토크가 필요보다 작다: 가속 국면이면 덜 밈, 감속 국면이면 너무 세게 제동 — 둘 다 덜 돈다)")
for T in (1.0, 1.2, 1.4):
    a_pk = 6 * (math.pi / 2) / T ** 2
    print(f"rot_s {T}: 최고 각가속도 {a_pk:.2f} → 가속 쪽 필요 {(I_R * a_pk + M_R) / C * 1e3:.0f} mN·m, 감속 쪽 {(I_R * a_pk - M_R) / C * 1e3:.0f} mN·m")
