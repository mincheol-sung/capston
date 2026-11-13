#!/usr/bin/env python3
"""IMM 운동모델 파라미터를 NIS 로 역산한다. 정답 궤적(ground truth)이 필요 없다.

    # 도구 자체 검증 — 아는 값으로 궤적을 만들어 되찾아지는지
    python3 tools/nis_fit.py --sim 1.0          # CV 만 있는 궤적
    python3 tools/nis_fit.py --sim-imm          # CV/CT 가 섞인 궤적

    # 실측 bag (ros2 bag record /detections_fused, 사람 기동 90초+)
    python3 tools/nis_fit.py --bag ~/bags/sigma_a_D1_run1

무엇을 내는가
-------------
    sigma_a       CV mode 의 CWNA 세기        [m/s^1.5]
    sigma_a_ct    CT mode 의 CWNA 세기        [m/s^1.5]
    mean_sojourn_cv / _ct   mode 체류 시간    [s]        (진단 등급)

왜 fused NIS 로는 안 되는가
---------------------------
예전 이 도구는 moment-matched fused innovation 하나로 sigma_a 를 냈다. IMM 이
들어오면서 그 전제가 깨졌다 — fused S 는 sigma_a^2 에 비례하지 않는다.
moment_match 가 CT mode 의 P (세기가 sigma_a_ct 로 고정) 와 mode 간 벌어짐
outer(delta, delta) 를 함께 넣기 때문이다. 실제로 재면:

    참 sigma_a   fused 로 추정     CT 죽이고 추정
      0.3          0.248 (-17.2%)    0.297 (-0.9%)
      1.0          1.048 ( +4.8%)    0.988 (-1.2%)
      3.0          3.206 ( +6.9%)    3.005 (+0.2%)

부호가 sigma_a 와 sigma_a_ct 의 대소를 따라 뒤집힌다. 그래서 mode-conditioned
NIS 를 mode 확률로 가중해 두 파라미터를 따로 푼다 (core/nis.py 의
ModeNisAccumulator). 두 축을 번갈아 고정점 반복한다.

한 번에 안 맞는다. 필터가 틀린 값으로 돌면 그 자신의 속도 추정 잡음이
innovation 에 섞이기 때문이다. 축약사상이라 반복하면 수렴한다.

bag 모드가 base_link 좌표를 그대로 쓴다는 점에 주의. odom TF 를 같이 녹화해
변환하는 것이 정확하지만, 로봇이 정지해 있을 때 녹화한 bag 이면 차이가 없다.
움직이면서 녹화했으면 --bag 결과는 로봇 자기운동까지 흡수하므로 과대추정이
된다. 그 경우 정지 상태에서 다시 녹화하는 편이 빠르다.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from object_tracker.core.config import TrackerConfig               # noqa: E402
from object_tracker.core.imm import MODE_CT, MODE_CV               # noqa: E402
from object_tracker.core.motion_models import (                    # noqa: E402
    ct_transition_and_jacobian,
)
from object_tracker.core.nis import (                              # noqa: E402
    ModeNisAccumulator,
    NisAccumulator,
    SojournAccumulator,
    gated_mean,
)
from object_tracker.core.tracker import MultiObjectTracker         # noqa: E402

DT = 1.0 / 12.0
MODE_NAME = {MODE_CV: 'CV', MODE_CT: 'CT'}


def _cwna_cholesky(sigma_a, dt=DT):
    """축 하나의 [p, v] CWNA 공분산 Cholesky. Q = [[q d^3/3, q d^2/2], ...]."""
    q = float(sigma_a) ** 2
    return np.linalg.cholesky(np.array([[q * dt ** 3 / 3, q * dt ** 2 / 2],
                                        [q * dt ** 2 / 2, q * dt]]))


# ── 측정 소스 ────────────────────────────────────────────────────────── #

def sim_frames(sigma_a_true, n_seg=200, steps=90, seed=0):
    """CV 만 있는 진짜 CWNA 궤적 + C2 실측 R(d). (stamp, [z], [R]) 를 yield.

    구간을 짧게 끊는 이유: velocity random walk 라 길게 두면 물체가 R(d) 의
    유효구간(1~9m) 밖으로 나간다. sigma_a=1.0 이면 333초 뒤 속도 sd 가
    sqrt(1.0^2 * 333) = 18 m/s 다.

    이 궤적에는 CT 성분이 없으므로 sigma_a_ct 회수를 검증하지 못한다.
    그 검증은 sim_imm_frames 가 한다.
    """
    from detection_floor import fused_sigmas

    rng = np.random.default_rng(seed)
    L = _cwna_cholesky(sigma_a_true)
    for _ in range(n_seg):
        s = np.array([[4.0, 0.0], [0.0, 0.0]])       # 축별 [p, v]
        t = 0.0
        yield None                                    # 구간 경계 = 트래커 리셋
        for _ in range(steps):
            t += DT
            for ax in (0, 1):
                s[ax] = ([s[ax, 0] + DT * s[ax, 1], s[ax, 1]]
                         + L @ rng.normal(size=2))
            p = s[:, 0].copy()
            d = float(np.hypot(*p))
            if not 1.0 <= d <= 9.0:
                break
            yield _measure(p, d, rng)


def sim_imm_frames(sigma_a=0.6, sigma_a_ct=1.2, tau_cv=3.0, tau_ct=1.0,
                   sigma_omega_ct=0.5, omega_enter=1.0, speed0=1.2,
                   n_seg=400, steps=110, seed=0):
    """CV <-> CT 가 CTMC 로 오가는 진짜 궤적. sigma_a_ct 회수를 검증한다.

    CV 구간은 sigma_a 세기의 velocity random walk 에 omega = 0.
    CT 구간은 coordinated turn (필터와 같은 이산화 ct_transition_and_jacobian)
    에 sigma_a_ct 세기의 CWNA 와 sigma_omega_ct 세기의 turn-rate random walk.
    CT 진입 시 omega ~ N(0, omega_enter) 를 뽑는다 — omega_enter=1.0 rad/s 면
    90도 선회에 1.57 초로 사람의 실제 선회에 해당한다.

    초기 속도를 보행 속도로 주는 이유: CT 는 속도를 회전시키는 모델이라
    v = 0 이면 CV 와 구분되지 않아 sigma_a_ct 가 식별되지 않는다.
    """
    from detection_floor import fused_sigmas          # noqa: F401 (문서용)

    rng = np.random.default_rng(seed)
    L = {MODE_CV: _cwna_cholesky(sigma_a),
         MODE_CT: _cwna_cholesky(sigma_a_ct)}
    rate = {MODE_CV: 1.0 / float(tau_cv), MODE_CT: 1.0 / float(tau_ct)}

    for _ in range(n_seg):
        heading = rng.uniform(0.0, 2.0 * math.pi)
        x = np.array([4.0, 0.0,
                      speed0 * math.cos(heading), speed0 * math.sin(heading),
                      0.0])
        mode = MODE_CV
        t = 0.0
        yield None
        for _ in range(steps):
            t += DT
            # 1) CTMC mode 전이. P(전이) = 1 - exp(-rate dt).
            if rng.random() < -math.expm1(-rate[mode] * DT):
                mode = MODE_CT if mode == MODE_CV else MODE_CV
                x[4] = rng.normal(0.0, omega_enter) if mode == MODE_CT else 0.0

            # 2) 결정론적 전파. CT 는 필터와 같은 이산화를 쓴다.
            if mode == MODE_CV:
                x = np.array([x[0] + DT * x[2], x[1] + DT * x[3],
                              x[2], x[3], 0.0])
            else:
                x = ct_transition_and_jacobian(x, DT)[0].copy()
                x[4] += float(sigma_omega_ct) * math.sqrt(DT) * rng.normal()

            # 3) CWNA 잡음. 축별 [p, v] 에 같은 세기로.
            for ax in (0, 1):
                noise = L[mode] @ rng.normal(size=2)
                x[ax] += noise[0]
                x[2 + ax] += noise[1]

            p = x[:2].copy()
            d = float(np.hypot(*p))
            if not 1.0 <= d <= 9.0:
                break
            yield _measure(p, d, rng)


def _measure(p, d, rng):
    """C2 실측 R(d) 로 관측을 만든다. 오차 타원을 시선 방향으로 회전시킨다."""
    from detection_floor import fused_sigmas

    sr, st = fused_sigmas(d, camera=(d <= 6.5))
    th = math.atan2(p[1], p[0])
    C = np.array([[math.cos(th), -math.sin(th)],
                  [math.sin(th), math.cos(th)]])
    R = C @ np.diag([sr ** 2, st ** 2]) @ C.T
    return 0.0, [p + np.linalg.cholesky(R) @ rng.normal(size=2)], [R]


def _stamped(frames):
    """시각을 12Hz 로 다시 매긴다. _measure 가 0.0 을 넣어 두기 때문."""
    t = 0.0
    for item in frames:
        if item is None:
            yield None
            continue
        t += DT
        yield (t, item[1], item[2])


def bag_frames(path, topic='/detections_fused'):
    """rosbag2 에서 DetectedObjectArray 를 읽는다. ROS 환경이 필요하다."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from amr_interfaces.msg import DetectedObjectArray
    except ImportError as exc:
        raise SystemExit(
            f'bag 모드는 ROS 환경이 필요하다: {exc}\n'
            '  source /opt/ros/*/setup.bash && source install/setup.bash')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path),
                rosbag2_py.ConverterOptions('', ''))
    yield None
    while reader.has_next():
        tp, data, _ = reader.read_next()
        if tp != topic:
            continue
        msg = deserialize_message(data, DetectedObjectArray)
        zs, Rs = [], []
        for o in msg.objects:
            R = np.array(o.position_covariance, dtype=float).reshape(2, 2)
            if not np.any(R):
                continue                  # 미기입은 R 을 모르므로 표본에서 뺀다
            zs.append(np.array([o.position.x, o.position.y]))
            Rs.append(R)
        if zs:
            t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            yield t, zs, Rs


# ── 반복 ─────────────────────────────────────────────────────────────── #

def one_pass(frames, sigma_a, sigma_a_ct, burn_in=3):
    """주어진 (sigma_a, sigma_a_ct) 로 전부 돌려 누적기 셋을 채운다."""
    cfg = TrackerConfig()
    cfg.sigma_a = float(sigma_a)
    cfg.sigma_a_ct = float(sigma_a_ct)
    fused = NisAccumulator(gate=cfg.gate_chi2, burn_in=burn_in)
    modes = ModeNisAccumulator(gate=cfg.gate_chi2, burn_in=burn_in, n_modes=3)
    sojourn = SojournAccumulator(n_modes=3)

    def new_tracker():
        return MultiObjectTracker(cfg, nis=fused, mode_nis=modes,
                                  sojourn=sojourn)

    trk = new_tracker()
    for item in frames:
        if item is None:                  # 구간 경계
            # 트랙 id 가 0 부터 다시 나므로 진행 중인 run 을 먼저 확정한다.
            for t in trk.tracks:
                sojourn.end_track(t.id)
            trk = new_tracker()
            continue
        trk.step(*item)
    for t in trk.tracks:
        sojourn.end_track(t.id)
    return fused, modes, sojourn


def fit(make_frames, sigma_a0=1.0, sigma_a_ct0=1.0, iters=8, burn_in=3,
        tolerance=0.01):
    """두 축 동시 고정점 반복. make_frames() 는 매번 새 generator 를 준다."""
    sa, sact = float(sigma_a0), float(sigma_a_ct0)
    print(f'{"iter":>4} {"sa":>8} {"sa_ct":>8} | '
          f'{"c_CV":>7} {"NIS_CV":>7} {"N_CV":>8} | '
          f'{"c_CT":>7} {"NIS_CT":>7} {"N_CT":>8} | '
          f'{"mu_CT":>6} {"기각":>6}')
    result = None
    for k in range(iters):
        fused, modes, sojourn = one_pass(make_frames(), sa, sact, burn_in)
        next_sa = modes.sigma_estimate(MODE_CV, sa)
        next_sact = modes.sigma_estimate(MODE_CT, sact)
        if next_sa is None or next_sact is None:
            print(f'{k:4d} {sa:8.4f} {sact:8.4f}   표본 없음 — '
                  '트랙이 안 만들어졌다')
            return None
        mu = modes.mean_probability
        print(f'{k:4d} {sa:8.4f} {sact:8.4f} | '
              f'{modes.scale(MODE_CV):7.3f} {modes.mean(MODE_CV):7.3f} '
              f'{modes.weight[MODE_CV]:8.1f} | '
              f'{modes.scale(MODE_CT):7.3f} {modes.mean(MODE_CT):7.3f} '
              f'{modes.weight[MODE_CT]:8.1f} | '
              f'{mu[MODE_CT]:6.3f} {fused.reject_fraction * 100:5.2f}%')
        converged = (abs(next_sa / sa - 1.0) < tolerance
                     and abs(next_sact / sact - 1.0) < tolerance)
        sa, sact = next_sa, next_sact
        result = (sa, sact, modes, sojourn, fused)
        if converged:
            break
    return result


def _report(result):
    sa, sact, modes, sojourn, fused = result
    print()
    print(f'  목표 평균 NIS {gated_mean(1.0, modes.gate):.3f} '
          f'(게이트 절단 때문에 2.000 이 아니다), 목표 c = 1.000, 기각률 1.0%')
    print()
    print(f'  sigma_a     = {sa:.3f}  [m/s^1.5]  '
          f'(+-{modes.relative_error(MODE_CV) * 100:.1f}%)   CV mode')
    print(f'  sigma_a_ct  = {sact:.3f}  [m/s^1.5]  '
          f'(+-{modes.relative_error(MODE_CT) * 100:.1f}%)   CT mode')
    print()
    print('  체류 시간 (진단 등급 — soft 와 hard 가 크게 다르면 mu 가 확신을')
    print('  못 내는 것이고, 그때는 체류시간보다 위 두 값을 먼저 봐야 한다):')
    for mode in (MODE_CV, MODE_CT):
        print(f'    mean_sojourn_{MODE_NAME[mode].lower()} = '
              f'soft {sojourn.soft_sojourn(mode):6.3f} s   '
              f'hard {sojourn.hard_sojourn(mode):6.3f} s')
    print()
    print(f'  평균 mode 확률  CV {modes.mean_probability[MODE_CV]:.3f}  '
          f'CT {modes.mean_probability[MODE_CT]:.3f}')
    print(f'  fused NIS (참고): 평균 {fused.mean:.3f}  c {fused.scale:.3f}  '
          f'기각 {fused.reject_fraction * 100:.2f}%')


def main():
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--sim', type=float, metavar='SIGMA_A_TRUE',
                     help='CV 만 있는 궤적으로 도구를 검증한다')
    src.add_argument('--sim-imm', action='store_true',
                     help='CV/CT 가 섞인 궤적으로 도구를 검증한다')
    src.add_argument('--bag', metavar='PATH', help='rosbag2 디렉터리')
    ap.add_argument('--topic', default='/detections_fused')
    ap.add_argument('--true-sa', type=float, default=0.6,
                    help='--sim-imm 의 참 sigma_a')
    ap.add_argument('--true-sa-ct', type=float, default=1.2,
                    help='--sim-imm 의 참 sigma_a_ct')
    ap.add_argument('--true-tau-cv', type=float, default=3.0)
    ap.add_argument('--true-tau-ct', type=float, default=1.0)
    ap.add_argument('--start', type=float, default=1.0, help='초기 sigma_a')
    ap.add_argument('--start-ct', type=float, default=1.0,
                    help='초기 sigma_a_ct')
    ap.add_argument('--iters', type=int, default=8)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--burn-in', type=int, default=3,
                    help='트랙 생성 직후 제외할 update 수 (P 가 정상상태가 '
                         '아니라 S 를 과대평가한다)')
    a = ap.parse_args()

    if a.sim is not None:
        print(f'=== 검증 모드 (CV 궤적): 참 sigma_a = {a.sim} ===')
        print('    CT 성분이 없으므로 sigma_a_ct 는 검증 대상이 아니다.')
        make = lambda: _stamped(sim_frames(a.sim, seed=a.seed))   # noqa: E731
    elif a.sim_imm:
        print(f'=== 검증 모드 (CV/CT 궤적): 참 sigma_a = {a.true_sa}, '
              f'sigma_a_ct = {a.true_sa_ct}, '
              f'tau_cv = {a.true_tau_cv}, tau_ct = {a.true_tau_ct} ===')
        make = lambda: _stamped(sim_imm_frames(   # noqa: E731
            a.true_sa, a.true_sa_ct, a.true_tau_cv, a.true_tau_ct,
            seed=a.seed))
    else:
        print(f'=== bag: {a.bag}  topic {a.topic} ===')
        make = lambda: bag_frames(a.bag, a.topic)                 # noqa: E731

    result = fit(make, a.start, a.start_ct, a.iters, a.burn_in)
    if result is None:
        return
    _report(result)
    sa, sact = result[0], result[1]

    if a.sim is not None:
        print()
        print(f'  sigma_a  추정 {sa:.4f}  참 {a.sim:.4f}  '
              f'오차 {(sa / a.sim - 1) * 100:+.1f}%')
    elif a.sim_imm:
        print()
        print(f'  sigma_a     추정 {sa:.4f}  참 {a.true_sa:.4f}  '
              f'오차 {(sa / a.true_sa - 1) * 100:+.1f}%')
        print(f'  sigma_a_ct  추정 {sact:.4f}  참 {a.true_sa_ct:.4f}  '
              f'오차 {(sact / a.true_sa_ct - 1) * 100:+.1f}%')
    else:
        print()
        print('  core/config.py 의 기본값과 config/tracker.yaml 을 이 값으로.')
        print('  주석의 [TODO] 를 [실측] 으로 바꾸고 날짜·bag 이름을 남길 것.')
        print('  바꾼 뒤 tools/detection_floor.py 를 다시 돌려 판정 하한을 확인할 것.')


if __name__ == '__main__':
    main()
