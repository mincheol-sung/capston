#!/usr/bin/env python3
"""STATIC / DYNAMIC 판별을 시나리오로 돌려 보고 전환 지연을 잰다.

    PYTHONPATH=. python3 tools/motion_state_demo.py

판별식은 속도의 chi^2 검정이다 (core/track.py).

    d^2 = v_hat^T P_vv^-1 v_hat  ~  chi^2_2      H0: v = 0

고정 속도 문턱(V_static, V_dynamic)을 쓰지 않는 이유는 그것이 두 개의
미실측 상수를 만들기 때문이다. chi^2 로 두면 문턱이 분위수라 유도되고
(chi^2_2(p) = -2 ln(1-p)), 실측해야 할 값이 sigma_a 하나로 줄어든다.

hysteresis 는 별도 장치 없이 같은 통계량 안에서 나온다 — 진입 0.99(9.21),
이탈 0.90(4.61) 로 비대칭이라 그 사이 [4.61, 9.21] 이 불감대가 되고, 이
구간에서는 어느 카운터도 건드리지 않는다.

여기서 재는 것은 두 가지다.
  1. 진짜 정지 물체가 STATIC 을 유지하는가 (오경보)
  2. 걷다 멈추고 다시 걸을 때 트랙 ID 가 끊기지 않고 상태만 바뀌는가,
     그리고 전환에 몇 프레임 걸리는가 (지연)
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from detection_floor import fused_sigmas                       # noqa: E402
from object_tracker.core.config import TrackerConfig           # noqa: E402
from object_tracker.core.track import MOTION_NAME               # noqa: E402
from object_tracker.core.tracker import MultiObjectTracker     # noqa: E402

DT = 1.0 / 12.0


def measure(p, rng):
    """C2 실측 R(d) 로 관측을 만든다. 극좌표축이 R 의 고유축이다."""
    d = max(float(np.hypot(*p)), 0.5)
    sr, st = fused_sigmas(d, camera=(d <= 6.5))
    th = math.atan2(p[1], p[0])
    C = np.array([[math.cos(th), -math.sin(th)],
                  [math.sin(th), math.cos(th)]])
    R = C @ np.diag([sr ** 2, st ** 2]) @ C.T
    return p + np.linalg.cholesky(R) @ rng.normal(size=2), R


def speed_profile(t_ramp, v_walk=1.2):
    """보행 3s -> t_ramp 초에 걸쳐 감속 -> 정지 3s -> t_ramp 가속 -> 보행 3s.

    계단 입력을 쓰면 안 된다. 1.2 m/s 를 한 프레임(83.3ms)에 죽이면
    a = 14.4 m/s^2 인데, sigma_a=1.0 의 게이트가 견디는 한계가 7.76 m/s^2 라
    association 이 끊기고 트랙이 새로 생긴다. 사람은 그렇게 못 멈춘다 —
    1.2 m/s 를 0.5s 에 세우면 2.40 m/s^2 다.
    """
    n_ramp = max(int(round(t_ramp / DT)), 1)
    seq = ([('보행', v_walk)] * 36
           + [('감속', v_walk * (1.0 - (i + 1) / n_ramp)) for i in range(n_ramp)]
           + [('정지', 0.0)] * 36
           + [('가속', v_walk * (i + 1) / n_ramp) for i in range(n_ramp)]
           + [('보행', v_walk)] * 36)
    return seq


def walk_stop_walk(cfg, t_ramp=0.5, seed=0, v_walk=1.2):
    """사람: 보행 -> 감속 -> 정지 -> 가속 -> 보행. 상자: 계속 정지."""
    rng = np.random.default_rng(seed)
    trk = MultiObjectTracker(cfg)
    person = np.array([3.0, -1.8])
    box = np.array([5.0, 1.2])

    log, t, k = [], 0.0, 0
    for name, speed in speed_profile(t_ramp, v_walk):
        t += DT
        k += 1
        person = person + np.array([0.0, speed]) * DT
        zs, Rs = zip(*(measure(p, rng) for p in (person, box)))
        trk.step(t, list(zs), list(Rs))
        log.append((k, t, name, speed,
                    {o.id: (o.motion, o.chi2_v, o.kf.speed, o.lifecycle)
                     for o in trk.tracks}))
    return trk, log


def ramp_sweep(cfg, v_walk=1.2, seeds=8):
    """감속 시간을 줄여 가며 트랙이 끊어지는 경계를 찾는다."""
    rows = []
    for t_ramp in (1.0, 0.75, 0.5, 0.35, 0.25, 0.17, DT):
        broke = 0
        for s in range(seeds):
            trk, _ = walk_stop_walk(cfg, t_ramp, seed=s, v_walk=v_walk)
            broke += (trk.next_id > 2)
        rows.append((t_ramp, v_walk / t_ramp, broke, seeds))
    return rows


def transition_delay(log, track_id=0):
    """상태가 바뀐 프레임과, 실제 운동이 바뀐 프레임의 차이."""
    out, prev_speed, change_k = [], None, None
    for k, _, _, speed, st in log:
        if prev_speed is not None and speed != prev_speed:
            change_k = k
        prev_speed = speed
        if track_id not in st:
            continue
        m = st[track_id][0]
        if out and out[-1][1] == m:
            continue
        out.append((k, m, change_k))
    return out


def main():
    cfg = TrackerConfig()
    print('=== 설정 ===')
    print(f'  sigma_a = {cfg.sigma_a} m/s^1.5   sigma_v0 = {cfg.sigma_v0} m/s')
    print(f'  DYNAMIC probability >= {cfg.dynamic_enter_probability}, '
          f'{cfg.dynamic_confirm_time}s')
    print(f'  STATIC probability >= {cfg.static_enter_probability}, '
          f'{cfg.static_confirm_time}s')
    print()

    t_ramp = 0.5
    trk, log = walk_stop_walk(cfg, t_ramp)

    print(f'=== 시나리오: 사람(#0) 보행 1.2 m/s -> {t_ramp}s 감속'
          f'({1.2 / t_ramp:.2f} m/s^2) -> 정지 3s -> 재가속 -> 보행 ===')
    print('    상자(#1) 은 내내 정지')
    print(f'{"k":>4} {"t[s]":>6} {"국면":>6} {"v[m/s]":>7} | '
          f'{"#0 상태":>9} {"chi2_v":>9} {"|v|":>7} | '
          f'{"#1 상태":>9} {"chi2_v":>9} {"|v|":>7}')
    n_ramp = max(int(round(t_ramp / DT)), 1)
    marks = ([1, 3, 5, 8, 20, 36]
             + [36 + n_ramp, 36 + n_ramp + 3, 36 + n_ramp + 6,
                36 + n_ramp + 10, 36 + n_ramp + 20, 36 + n_ramp + 36]
             + [36 + 2 * n_ramp + 36, 36 + 2 * n_ramp + 39,
                36 + 2 * n_ramp + 45, 36 + 2 * n_ramp + 60])
    for k, t, name, v, st in log:
        if k not in marks:
            continue
        row = f'{k:4d} {t:6.2f} {name:>6} {v:7.2f} |'
        for tid in (0, 1):
            if tid in st:
                m, c, s, _lc = st[tid]
                row += f' {MOTION_NAME[m]:>9} {c:9.2f} {s:7.3f} |'
            else:
                row += f' {"-":>9} {"-":>9} {"-":>7} |'
        print(row)

    print()
    print('=== 전환 지연 (#0 사람) ===')
    for k, m, change_k in transition_delay(log, 0):
        d = f'운동변화 k={change_k} 로부터 {k - change_k} 프레임 ' \
            f'({(k - change_k) * DT * 1000:.0f} ms)' if change_k else '트랙 생성'
        print(f'  k={k:4d}  -> {MOTION_NAME[m]:8s}  {d}')

    print()
    print('=== 상자(#1): 진짜 정지 물체가 흔들리지 않는가 ===')
    box_states = {}
    for _, _, _, _, st in log:
        if 1 in st:
            box_states[MOTION_NAME[st[1][0]]] = \
                box_states.get(MOTION_NAME[st[1][0]], 0) + 1
    print(f'  프레임별 상태 분포: {box_states}')
    chi = [st[1][1] for _, _, _, _, st in log if 1 in st]
    print(f'  chi2_v diagnostic only: {min(chi):.3f} ~ {max(chi):.3f}')

    print()
    print('=== 트랙 ID 연속성 ===')
    ids = sorted(t_.id for t_ in trk.tracks)
    print(f'  최종 트랙 {ids}, 총 생성 {trk.next_id}개 (물체 2개)')
    print('  -> 멈췄다 다시 걸어도 ID 가 안 바뀌면 tracker 를 정적용/동적용으로'
          ' 나누지 않은 설계가 의도대로 동작한 것이다.')
    assert trk.next_id == 2, 'ID 가 새로 생겼다'

    print()
    print('=== 감속이 얼마나 급하면 트랙이 끊어지는가 (시드 8개) ===')
    print(f'{"감속시간":>9} {"감속도":>10} {"트랙 끊김":>10}')
    for t_r, a, broke, n in ramp_sweep(cfg):
        note = ''
        if abs(t_r - DT) < 1e-9:
            note = '  <- 계단 입력. 사람은 이렇게 못 멈춘다'
        print(f'{t_r:8.2f}s {a:8.2f} m/s^2 {broke:6d}/{n}{note}')
    print('  참고: 이분탐색으로 잰 sigma_a=1.0 의 게이트 한계가 7.76 m/s^2 였다.')
    print('       사람의 실제 정지 감속도는 1.5~4.0 m/s^2 라 여유가 있다.')

    print()
    print('=== UNKNOWN 을 따로 두는 이유 ===')
    first = [(k, MOTION_NAME[st[0][0]], st[0][1])
             for k, _, _, _, st in log if 0 in st][:6]
    for k, m, c in first:
        print(f'  k={k}  {m:8s} chi2_v={c:8.3f}')
    print('  TENTATIVE 동안 P_vv = sigma_v0^2 = '
          f'{cfg.sigma_v0 ** 2:.2f} 로 열려 있어 chi^2 가 무조건 작게 나온다.')
    print('  그대로 판정하면 "속도를 아직 못 재서"가 "정지해 있어서"로 둔갑한다.')


if __name__ == '__main__':
    main()
