"""RoboClaw 통신 시간·지연 분해 측정 — ROS 를 거치지 않고 시리얼에 직접 붙는다.

왜 ROS 밖인가: velocity_step_test / wp2_step_logger 는 드라이버 노드를 거쳐 재므로
드라이버 대기(단일 스레드 executor 에서 명령 콜백이 50 Hz 읽기 루프 뒤에 줄 선다)·시리얼·
RoboClaw 내부 PI·모터·속도 계산 지연이 한 덩어리로 나온다 (plant_wheel.yaml 의 θ = 40 ms).
여기서는 모든 시리얼 트랜잭션에 host 시각을 찍어 그 덩어리를 나눈다.

    roboclaw_probe bench   [--fake]           A. 통신 왕복 시간·명령 지원 여부·주기 구성별 최대 Hz.  모터 안 돌림
    roboclaw_probe delay   [--fake] [--yes]   B. 계단 명령 + 엔코더 카운트/cmd18 속도/cmd30 속도 기록.  ★ 바퀴를 띄울 것
    roboclaw_probe analyze <delay.csv>        B 의 CSV 를 계단마다 FOPDT 로 맞춰 구동 지연과 측정 지연을 나눈다

기본 포트·주소·변환계수는 amr_bringup/config/base.yaml 과 같다 (인자로 바꿀 수 있다).
--fake 는 amr_base.roboclaw_fake 의 가짜 보드(구동 지연 15 ms, τ 90 ms, cmd18 창 30 ms)로 돈다 — 도구 자체 검증용.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple

DEFAULTS = dict(left_port='/dev/roboclaw_left', right_port='/dev/roboclaw_right',
                left_address=128, right_address=129, baudrate=115200, timeout=0.01,
                qpps_per_rad=210.08)


def s32(v: int) -> int:
    """Basicmicro 는 32-bit 필드를 unsigned 로 준다. 이 보드는 two's complement (roboclaw_mecanum_node 주석)."""
    v = int(v) & 0xFFFFFFFF
    return v - 0x100000000 if v >= 0x80000000 else v


# ───────────────────────────── 연결 ─────────────────────────────
@dataclass
class Board:
    name: str
    claw: object
    address: int


def driver_running() -> bool:
    """실행 파일이 드라이버인 프로세스만 센다.

    pgrep -f 는 명령줄 어디에든 그 글자가 있으면 잡는다 — 그 글자를 담은 셸 명령(grep, pgrep 자신,
    이 도구를 띄운 bash -c)까지 드라이버로 오인한다 (2026-10-22 실제로 그랬다). argv[0] 또는
    python 스크립트인 argv[1] 의 파일 이름이 정확히 roboclaw_mecanum_node 인 것만 본다.
    """
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                argv = f.read().split(b'\0')
        except OSError:
            continue
        if any(os.path.basename(a.decode(errors='replace')) == 'roboclaw_mecanum_node' for a in argv[:2]):
            return True
    return False


def open_boards(args) -> List[Board]:
    if args.fake:
        from amr_base.roboclaw_fake import FakeRoboClaw
        mk = lambda port, seed: FakeRoboClaw(port, args.baudrate, args.timeout, args.retries, seed=seed,
                                             supports_getspeeds=not args.fake_no_getspeeds)
        boards = [Board('left', mk('fake_left', 1), args.left_address),
                  Board('right', mk('fake_right', 2), args.right_address)]
    else:
        if driver_running():
            sys.exit('roboclaw_mecanum_node 가 떠 있다. 같은 포트에 두 프로세스가 쓰면 패킷이 섞인다 — 드라이버를 끄고 다시.')
        from basicmicro import Basicmicro
        boards = []
        for name, port, addr in (('left', args.left_port, args.left_address), ('right', args.right_port, args.right_address)):
            if not os.path.exists(port):
                sys.exit(f'{port} 가 없다 (USB 연결·udev 심링크 확인)')
            claw = Basicmicro(port, args.baudrate, timeout=args.timeout, retries=args.retries)
            if not claw.Open():
                sys.exit(f'{port} 를 열 수 없다')
            boards.append(Board(name, claw, addr))
    if args.fake:
        for b in boards:
            b.claw.Open()
    return boards


def call(fn: Callable, *a) -> Tuple[bool, object, float, float]:
    """(성공, 결과, 시작 시각, 끝 시각).  예외(미지원 명령의 타임아웃 포함)도 실패로 돌려준다."""
    t0 = time.monotonic()
    try:
        r = fn(*a)
        ok = (r is True) or (isinstance(r, tuple) and bool(r) and bool(r[0]))
    except Exception as exc:  # PacketTimeoutError 등
        r, ok = exc, False
    return ok, r, t0, time.monotonic()


def stop_all(boards: List[Board]):
    for b in boards:
        for _ in range(3):
            ok, *_ = call(b.claw.SpeedM1M2, b.address, 0, 0)
            if ok:
                break
        else:
            print(f'⚠ {b.name} 정지 명령 확인 실패 — 비상정지를 쓸 것', file=sys.stderr)


def pct(xs: List[float], q: float) -> float:
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(round(q / 100 * (len(xs) - 1))))] if xs else math.nan


# ───────────────────────────── A. bench ─────────────────────────────
READS = {
    'ReadSpeedM1 (18)': lambda c, a: c.ReadSpeedM1(a),
    'ReadSpeedM2 (19)': lambda c, a: c.ReadSpeedM2(a),
    'GetSpeeds (108)': lambda c, a: c.GetSpeeds(a),
    'ReadISpeedM1 (30)': lambda c, a: c.ReadISpeedM1(a),
    'GetEncoders (78)': lambda c, a: c.GetEncoders(a),
    'ReadPWMs (48)': lambda c, a: c.ReadPWMs(a),
    'ReadCurrents (49)': lambda c, a: c.ReadCurrents(a),
    'SpeedM1M2(0,0) (37)': lambda c, a: c.SpeedM1M2(a, 0, 0),
}


def cycle_ops(scheme: str) -> List[Callable]:
    """보드 하나가 한 주기에 하는 일 (드라이버 update + wheel_cmd 쓰기)."""
    if scheme == 'per_motor':
        return [READS['ReadSpeedM1 (18)'], READS['ReadSpeedM2 (19)'], READS['SpeedM1M2(0,0) (37)']]
    return [READS['GetSpeeds (108)'], READS['SpeedM1M2(0,0) (37)']]


def run_cycles(boards: List[Board], scheme: str, parallel: bool, n: int) -> Tuple[List[float], int]:
    ops = cycle_ops(scheme)
    times, fails = [], 0
    lock = threading.Lock()

    def board_cycle(b: Board):
        nonlocal fails
        for op in ops:
            ok, *_ = call(op, b.claw, b.address)
            if not ok:
                with lock:
                    fails += 1

    for _ in range(n):
        t0 = time.monotonic()
        if parallel:
            th = [threading.Thread(target=board_cycle, args=(b,)) for b in boards]
            for t in th:
                t.start()
            for t in th:
                t.join()
        else:
            for b in boards:
                board_cycle(b)
        times.append((time.monotonic() - t0) * 1e3)
    return times, fails


def cmd_bench(args):
    boards = open_boards(args)
    report: Dict[str, object] = {'when': datetime.now().isoformat(timespec='seconds'), 'fake': args.fake,
                                 'timeout_s': args.timeout, 'retries': args.retries, 'commands': {}, 'cycles': {}}
    try:
        for b in boards:
            ok, r, *_ = call(b.claw.ReadVersion, b.address)
            print(f'{b.name}: 펌웨어 {r[1] if ok else "읽기 실패"}')
            report[f'version_{b.name}'] = r[1] if ok else None
        print(f'\n[1] 명령별 왕복 시간  ({args.n} 회, 보드별)       p50 / p99 / max [ms]   실패')
        supported = {}
        for name, fn in READS.items():
            row = []
            for b in boards:
                ok0, *_ = call(fn, b.claw, b.address)          # 지원 여부 (미지원이면 timeout×retries 를 쓰고 실패)
                if not ok0:
                    row.append(f'{b.name}: 미지원/실패')
                    supported[(name, b.name)] = False
                    continue
                supported[(name, b.name)] = True
                ts, nf = [], 0
                for _ in range(args.n):
                    ok, _r, t0, t1 = call(fn, b.claw, b.address)
                    ts.append((t1 - t0) * 1e3)
                    nf += not ok
                row.append(f'{b.name}: {pct(ts, 50):5.2f} / {pct(ts, 99):5.2f} / {max(ts):6.2f}  {nf:3d}')
                report['commands'][f'{name}|{b.name}'] = dict(p50=pct(ts, 50), p99=pct(ts, 99), max=max(ts), fail=nf)
            print(f'    {name:22s} ' + '   '.join(row))
        print(f'\n[2] 한 주기 (보드마다 속도 읽기 + 속도 쓰기) {args.n} 회      p50 / p99 / max [ms]   실패   p99 기준 최대 Hz')
        for scheme in ('per_motor', 'combined'):
            if scheme == 'combined' and not all(supported.get(('GetSpeeds (108)', b.name)) for b in boards):
                print('    combined (GetSpeeds)     건너뜀 — 명령 108 미지원')
                continue
            for parallel in (False, True):
                ts, nf = run_cycles(boards, scheme, parallel, args.n)
                label = f'{scheme:9s} {"병렬(보드별 스레드)" if parallel else "순차"}'
                hz = 1000.0 / pct(ts, 99)
                print(f'    {label:28s} {pct(ts, 50):5.2f} / {pct(ts, 99):5.2f} / {max(ts):6.2f}   {nf:3d}   {hz:6.0f}')
                report['cycles'][label.strip()] = dict(p50=pct(ts, 50), p99=pct(ts, 99), max=max(ts), fail=nf, hz_p99=hz)
        print('\n판정: 100 Hz 는 주기 10 ms 안에 ROS 발행·odom 적분까지 들어가야 한다. p99 가 7 ms 아래인 구성을 고른다.'
              '\n      실패가 하나라도 있으면 그 주기는 timeout×retries'
              f' = {args.timeout * args.retries * 1e3:.0f} ms 를 쓴다 (base.yaml serial_timeout_s, 라이브러리 기본 retries=2).')
    finally:
        stop_all(boards)
    out = args.out or f'roboclaw_bench_{datetime.now():%Y%m%d_%H%M%S}.json'
    with open(out, 'w') as f:
        json.dump(report, f, indent=1, ensure_ascii=False)
    print(f'\n저장: {out}')


# ───────────────────────────── B. delay ─────────────────────────────
def schedule(amp_qpps: int, rest: float, hold: float, repeats: int) -> List[Tuple[float, int]]:
    """(시작 시각, qpps).  정지 → +A → 정지 → −A → 정지 … 를 repeats 번.  모든 상승 에지가 정지에서 출발한다."""
    out, t = [(0.0, 0)], rest
    for _ in range(repeats):
        for sgn in (1, -1):
            out += [(t, sgn * amp_qpps), (t + hold, 0)]
            t += hold + rest
    out.append((t, 0))
    return out


def cmd_delay(args):
    if abs(args.amp) > args.cap:
        sys.exit(f'--amp {args.amp} rad/s 가 상한 --cap {args.cap} 을 넘는다')
    amp_qpps = int(round(args.amp * args.qpps_per_rad))
    sch = schedule(amp_qpps, args.rest, args.hold, args.repeats)
    total = sch[-1][0]
    print(f'계단 ±{args.amp} rad/s (±{amp_qpps} qpps), 유지 {args.hold} s, 정지 {args.rest} s, {args.repeats} 회 → {total:.1f} s'
          f'\n읽기: GetEncoders + ReadSpeedM1/M2{" + ReadISpeedM1/M2" if args.ispeed else ""}, 목표 {args.rate:.0f} Hz')
    if not args.fake and not args.yes:
        print('\n★ 네 바퀴가 바닥에서 떠 있어야 한다 (받침대). 바퀴가 실제로 돈다. 비상정지를 손에 둘 것.')
        if input('바퀴를 띄웠으면 LIFTED 를 입력: ').strip() != 'LIFTED':
            sys.exit('취소')
    boards = open_boards(args)
    out = args.out or f'roboclaw_delay_{datetime.now():%Y%m%d_%H%M%S}.csv'
    rows: List[Tuple] = []
    period = 1.0 / args.rate
    overruns = 0
    try:
        stop_all(boards)
        time.sleep(0.5)
        t_start = time.monotonic()
        k_sched, k = 0, 0
        while True:
            t_cycle = t_start + k * period
            now = time.monotonic()
            if now < t_cycle:
                time.sleep(t_cycle - now)
            elif now - t_cycle > period:
                overruns += 1
            rel = time.monotonic() - t_start
            if rel >= total:
                break
            while k_sched + 1 < len(sch) and rel >= sch[k_sched + 1][0]:
                k_sched += 1
                q = sch[k_sched][1]
                for b in boards:
                    ok, _r, t0, t1 = call(b.claw.SpeedM1M2, b.address, q, q)
                    rows.append(((t0 + t1) / 2 - t_start, b.name, 'cmd', 0, q, (t1 - t0) * 1e3, int(ok)))
            for b in boards:
                ok, r, t0, t1 = call(b.claw.GetEncoders, b.address)
                if ok:
                    tm = (t0 + t1) / 2 - t_start
                    rows.append((tm, b.name, 'enc', 1, s32(r[1]), (t1 - t0) * 1e3, 1))
                    rows.append((tm, b.name, 'enc', 2, s32(r[2]), (t1 - t0) * 1e3, 1))
                for m, fn in ((1, b.claw.ReadSpeedM1), (2, b.claw.ReadSpeedM2)):
                    ok, r, t0, t1 = call(fn, b.address)
                    if ok:
                        rows.append(((t0 + t1) / 2 - t_start, b.name, 'spd', m, s32(r[1]), (t1 - t0) * 1e3, 1))
                if args.ispeed:
                    for m, fn in ((1, b.claw.ReadISpeedM1), (2, b.claw.ReadISpeedM2)):
                        ok, r, t0, t1 = call(fn, b.address)
                        if ok:
                            rows.append(((t0 + t1) / 2 - t_start, b.name, 'isp', m, s32(r[1]), (t1 - t0) * 1e3, 1))
            k += 1
    except KeyboardInterrupt:
        print('\n중단 — 정지 명령을 보낸다')
    finally:
        stop_all(boards)
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'board', 'kind', 'motor', 'value', 'io_ms', 'ok'])
        w.writerows(rows)
    print(f'저장: {out}  ({len(rows)} 행, 루프 {k} 회 = 실제 {k / max(total, 1e-9):.0f} Hz, 목표보다 늦은 주기 {overruns})'
          '\n      (목표보다 느려도 분석에는 문제없다 — 시각을 트랜잭션마다 찍는다. 빠르게 하려면 --no-ispeed)')
    if not args.no_analyze:
        analyze(out)


# ───────────────────────────── analyze ─────────────────────────────
def _fit_edge(t: List[float], y: List[float], t0: float, A: float):
    """계단 하나를 FOPDT 속도 응답으로 맞춘다.  y = y0 + K A (1 − e^{−x/τ}),  x = max(0, t − t0 − θ)."""
    import numpy as np
    from scipy.optimize import least_squares
    t, y = np.asarray(t), np.asarray(y, float)

    def model(p):
        th, tau, K, c0 = p
        x = np.clip(t - t0 - th, 0.0, None)
        return c0 + K * A * (1.0 - np.exp(-x / tau))

    best = None
    c_init = float(y[t < t0].mean()) if np.any(t < t0) else 0.0
    for th0 in (0.0, 0.01, 0.02, 0.04, 0.06):
        for tau0 in (0.01, 0.03, 0.08, 0.15):
            r = least_squares(lambda p: model(p) - y, [th0, tau0, 1.0, c_init],
                              bounds=([-0.02, 0.001, -10, -np.inf], [0.3, 1.0, 10, np.inf]))
            if best is None or r.cost < best.cost:
                best = r
    th, tau, K, _ = best.x
    return dict(theta=th, tau=tau, K=K, rms_rel=float(np.sqrt(np.mean(best.fun ** 2))) / max(abs(K * A), 1e-9))


def _count_speed(samples: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """연속한 카운트 샘플의 차분 / 시각 차 → (가운데 시각, counts/s).  평균 창이 없는 실제 바퀴 속도.
    한 샘플 간격(≈ 루프 주기)의 차분이라 그 절반만큼 시각을 가운데로 둔다."""
    out = []
    for (t1, c1), (t2, c2) in zip(samples, samples[1:]):
        if t2 - t1 > 1e-4:
            out.append(((t1 + t2) / 2, (c2 - c1) / (t2 - t1)))
    return out


LABEL = {'encv': '실제(카운트 미분)', 'spd': 'cmd18', 'isp': 'cmd30'}


def analyze(path: str):
    data: Dict[Tuple[str, str, int], List[Tuple[float, float]]] = {}
    cmds: Dict[str, List[Tuple[float, int]]] = {}
    with open(path) as f:
        for r in csv.DictReader(f):
            if r['kind'] == 'cmd':
                cmds.setdefault(r['board'], []).append((float(r['t']), int(float(r['value']))))
            else:
                data.setdefault((r['board'], r['kind'], int(r['motor'])), []).append((float(r['t']), float(r['value'])))
    for (board, kind, motor), s in list(data.items()):
        if kind == 'enc':
            data[(board, 'encv', motor)] = _count_speed(s)
    kinds = [k for k in ('encv', 'spd', 'isp') if any(key[1] == k for key in data)]
    print(f'\n{path}\n계단(정지에서 출발)마다 FOPDT 를 맞춘 결과의 중앙값 [ms].  encv = 엔코더 카운트 미분 = 평균 창이 없는 실제 바퀴 속도'
          f'\n{"보드·모터":10s} {"에지":>4s}' + ''.join(f'  {k + " θ":>8s} {k + " τ":>7s}' for k in kinds)
          + '   encv θ 범위   encv τ 범위' + '   spd−encv(θ+τ)' + ('  isp−encv(θ+τ)' if 'isp' in kinds else '')
          + '   이득 spd/encv')
    summary = {k: [] for k in kinds}
    for board, cl in sorted(cmds.items()):
        # 첫 기록 명령 앞은 정지다 (delay 가 시작 전에 stop_all 을 보낸다)
        edges = [(cl[i][0], cl[i][1], cl[i + 1][0] if i + 1 < len(cl) else math.inf)
                 for i in range(len(cl)) if (i == 0 or cl[i - 1][1] == 0) and cl[i][1] != 0]
        for motor in (1, 2):
            res = {k: [] for k in kinds}
            for t0, q, t_next in edges:
                for k in kinds:
                    seg = [(t, v) for t, v in data.get((board, k, motor), []) if t0 - 0.1 <= t <= min(t_next, t0 + 1.2)]
                    if len(seg) >= 20:
                        res[k].append(_fit_edge([p[0] for p in seg], [p[1] for p in seg], t0, float(q)))
            if not res[kinds[0]]:
                continue
            med = {k: (statistics.median(x['theta'] for x in res[k]) * 1e3, statistics.median(x['tau'] for x in res[k]) * 1e3,
                       statistics.median(x['K'] for x in res[k])) for k in kinds if res[k]}
            line = f'{board:5s} M{motor}    {len(res[kinds[0]]):4d}' + ''.join(
                f'  {med[k][0]:8.1f} {med[k][1]:7.1f}' if k in med else f'  {"-":>8s} {"-":>7s}' for k in kinds)
            th = [x['theta'] * 1e3 for x in res['encv']]
            ta = [x['tau'] * 1e3 for x in res['encv']]
            line += f'   {min(th):5.1f}–{max(th):5.1f}  {min(ta):5.1f}–{max(ta):5.1f}'
            for k in ('spd', 'isp'):
                if k in med:
                    line += f'   {med[k][0] + med[k][1] - med["encv"][0] - med["encv"][1]:+12.1f}'
            if 'spd' in med:
                line += f'   {med["spd"][2] / med["encv"][2]:8.3f}'
            print(line)
            for k in kinds:
                summary[k] += res[k]
    io = [float(r['io_ms']) for r in csv.DictReader(open(path)) if r['kind'] == 'cmd']
    print('\n해석'
          '\n  encv θ, τ      명령을 쓴 시각 → 실제 바퀴 속도.  RoboClaw 내부 PI·PWM·모터 (측정 평균 없음).'
          '\n                 바퀴를 띄운 시험이면 차체 관성이 빠져 있다 — 바닥 τ 는 더 크다'
          '\n  spd−encv       cmd 18 속도가 실제보다 늦는 양 (평균 체류시간 θ+τ 의 차) = RoboClaw 속도 계산 필터의 측정 지연.'
          '\n                 하중과 무관한 펌웨어 성질이다'
          '\n  isp            cmd 30 (counts/s, 짧은 창) — 지연은 작지만 잡음이 크다'
          '\n  → MPC 모델의 θ·τ 는 encv 쪽 (바닥에서 다시 잴 것). 측정 지연은 관측기(측정 경로) 쪽 문제다.'
          f'\n  명령 쓰기 왕복 p50 {pct(io, 50):.2f} ms (시각은 왕복의 가운데로 찍었다 — 그 절반이 시각 불확실성)')
    return summary


# ───────────────────────────── main ─────────────────────────────
def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='cmd', required=True)

    def common(sp):
        sp.add_argument('--fake', action='store_true', help='가짜 보드로 실행 (도구 검증)')
        sp.add_argument('--fake-no-getspeeds', action='store_true', help='가짜 보드가 명령 108 을 지원하지 않게')
        for k in ('left_port', 'right_port'):
            sp.add_argument('--' + k.replace('_', '-'), default=DEFAULTS[k])
        sp.add_argument('--left-address', type=int, default=DEFAULTS['left_address'])
        sp.add_argument('--right-address', type=int, default=DEFAULTS['right_address'])
        sp.add_argument('--baudrate', type=int, default=DEFAULTS['baudrate'])
        sp.add_argument('--timeout', type=float, default=DEFAULTS['timeout'], help='시리얼 timeout [s] (base.yaml serial_timeout_s)')
        sp.add_argument('--retries', type=int, default=2, help='Basicmicro 재시도 (드라이버는 라이브러리 기본 2)')
        sp.add_argument('--out', default=None)

    b = sub.add_parser('bench', help='A. 통신 시간 (모터 안 돌림)')
    common(b)
    b.add_argument('-n', type=int, default=300)
    d = sub.add_parser('delay', help='B. 지연 분해 (바퀴를 띄울 것)')
    common(d)
    d.add_argument('--amp', type=float, default=8.0, help='계단 크기 [rad/s]')
    d.add_argument('--cap', type=float, default=15.0, help='--amp 상한 [rad/s]')
    d.add_argument('--hold', type=float, default=1.5)
    d.add_argument('--rest', type=float, default=1.5)
    d.add_argument('--repeats', type=int, default=4)
    d.add_argument('--rate', type=float, default=100.0, help='읽기 루프 목표 [Hz]')
    d.add_argument('--qpps-per-rad', type=float, default=DEFAULTS['qpps_per_rad'])
    d.add_argument('--no-ispeed', dest='ispeed', action='store_false', help='cmd 30/31 읽기를 뺀다 (루프가 빨라진다)')
    d.add_argument('--yes', action='store_true', help='바퀴 띄움 확인을 건너뛴다')
    d.add_argument('--no-analyze', action='store_true')
    a = sub.add_parser('analyze', help='delay CSV 분석')
    a.add_argument('csv')
    args = p.parse_args(argv)
    {'bench': cmd_bench, 'delay': cmd_delay, 'analyze': lambda a: analyze(a.csv)}[args.cmd](args)


if __name__ == '__main__':
    main()
