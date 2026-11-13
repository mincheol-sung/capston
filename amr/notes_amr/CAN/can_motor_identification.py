#!/usr/bin/env python3
"""AK40-10 bench identification. Default: preview only; --run operates hardware.
Protocol assumptions match can_torque_feedback.py (AK40-10 V2 MIT, output shaft).
Run only with ALL wheels clear of the ground and chassis securely supported.
"""
import argparse
from collections import deque
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import socket
import struct
import time

WHEELS = {1: 'FL', 2: 'FR', 3: 'RL', 4: 'RR'}
P_RANGE, V_RANGE, T_RANGE = (-12.5, 12.5), (-45.5, 45.5), (-5., 5.)
SPEC = {'rated_torque_nm': 1.3, 'peak_torque_nm': 4.1, 'rated_current_a': 2.7,
        'peak_current_a': 7.3, 'rated_rpm': 370., 'no_load_rpm': 435.,
        'voltage_v': 24., 'gear_ratio': 10., 'motor_kt_nm_a': .056,
        'source': 'https://www.cubemars.com/product/ak40-10-robotic-actuator.html'}
ENABLE, DISABLE = bytes.fromhex('FFFFFFFFFFFFFFFC'), bytes.fromhex('FFFFFFFFFFFFFFFD')
FIELDS = ['cycle', 'motor_id', 'wheel', 'phase', 'phase_time_s', 'torque_plan_nm',
          'scheduled_time_s', 'tx_time_s', 'rx_time_s', 'unix_rx_s', 'lateness_ms',
          'response_ms', 'torque_requested_nm', 'torque_encoded_nm', 'rx_ok',
          'position_rad', 'velocity_rad_s', 'torque_fb_nm', 'temperature_c', 'error',
          'payload_hex', 'discarded_before_tx', 'speed_guard_active',
          'velocity_lpf_rad_s', 'torque_fb_lpf_nm', 'torque_cmd_lpf_nm',
          'velocity_sign_lpf', 'filter_reset', 'current_ideal_equivalent_a']


class LowPass:
    """First-order RC LPF, alpha=1-exp(-2*pi*fc*dt), actual RX intervals.
    Initialize from first sample; reset after gaps >50ms. Raw data never overwritten.
    """
    def __init__(self, cutoff_hz=5.):
        self.cutoff = cutoff_hz
        self.last_t = None
        self.values = None

    def update(self, t, values):
        reset = self.last_t is None or t <= self.last_t or t-self.last_t > .05
        if reset:
            self.values = dict(values)
        else:
            alpha = -math.expm1(-2.*math.pi*self.cutoff*(t-self.last_t))
            for key, value in values.items():
                self.values[key] += alpha*(value-self.values[key])
        self.last_t = t
        return dict(self.values), reset


def encode(v, limits, bits):
    lo, hi = limits
    return int(round((max(lo, min(hi, v)) - lo) * ((1 << bits)-1) / (hi-lo)))


def decode(v, limits, bits):
    lo, hi = limits
    return lo + v * (hi-lo) / ((1 << bits)-1)


def pack(torque, limit=None):
    p, v, t = encode(0., P_RANGE, 16), encode(0., V_RANGE, 12), encode(torque, T_RANGE, 12)
    if limit is not None:
        while abs(decode(t, T_RANGE, 12)) > limit:
            t += -1 if t > 2047 else 1
    return bytes([p >> 8, p & 255, v >> 4, (v & 15) << 4, 0, 0, t >> 8, t & 255]), decode(t, T_RANGE, 12)


def unpack(payload):
    return {'position_rad': decode((payload[1] << 8) | payload[2], P_RANGE, 16),
            'velocity_rad_s': decode((payload[3] << 4) | (payload[4] >> 4), V_RANGE, 12),
            'torque_fb_nm': decode(((payload[4] & 15) << 8) | payload[5], T_RANGE, 12),
            'temperature_c': payload[6]-40, 'error': payload[7]}


class CanBus:
    def __init__(self, interface, ids):
        # Prevent two instances of this script. Does not lock out other CAN programs.
        import fcntl
        key = hashlib.sha256(interface.encode()).hexdigest()[:12]
        self.lock = open('/tmp/amr_ident_' + key + '.lock', 'a')
        fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.settimeout(.02)
        self.sock.bind((interface,))
        self.ids = ids

    def send(self, mid, payload):
        self.sock.send(struct.pack('=IB3x8s', mid, 8, payload))

    def drain(self):
        count = 0
        while select.select([self.sock], [], [], 0)[0]:
            self.sock.recv(16)
            count += 1
            if count >= 1000:
                raise RuntimeError('CAN receive backlog excessive')
        return count

    def receive(self, mid, deadline):
        while time.monotonic() < deadline:
            if not select.select([self.sock], [], [], max(0., deadline-time.monotonic()))[0]:
                return None
            raw = self.sock.recv(16)
            rx = time.monotonic()
            if len(raw) != 16:
                continue
            cid, dlc, payload = struct.unpack('=IB3x8s', raw)
            if cid & 0x20000000:
                raise RuntimeError('CAN error frame')
            if cid & 0xC0000000 or (cid & 0x7FF) != mid or dlc != 8 or payload[0] != mid:
                continue
            return unpack(payload), rx, payload.hex()
        return None

    def close(self):
        self.sock.close()
        self.lock.close()


def stages(max_torque=.1):
    """Open-loop TORQUE ONLY. No speed target, PI, or speed-based brake."""
    out = [('baseline', 1., 0., 'zero')]
    for sign in (1., -1.):
        for fraction in (.2, .4, .6, .8, 1.):
            out.extend([('step', .6, sign*fraction*max_torque, 'torque'),
                        ('coast', 3., 0., 'zero'), ('stop', 4., 0., 'zero')])
        out.extend([('ramp', 1.5, sign*max_torque, 'ramp'),
                    ('coast', 3., 0., 'zero'), ('stop', 4., 0., 'zero')])
        for _ in range(6):
            out.extend([('pulse', .10, sign*.8*max_torque, 'torque'),
                        ('pulse_zero', .15, 0., 'zero')])
        out.extend([('coast', 3., 0., 'zero'), ('stop', 4., 0., 'zero')])
    return out


def planned_torque(control, target, elapsed, duration):
    return target*min(1., max(0., elapsed/duration)) if control == 'ramp' else (target if control == 'torque' else 0.)


def protection(data, max_speed, max_feedback_torque):
    for key in ('velocity_rad_s', 'torque_fb_nm', 'temperature_c', 'position_rad'):
        if not math.isfinite(data[key]):
            return 'non-finite feedback: '+key
    if data['error']:
        return 'motor error'
    if abs(data['velocity_rad_s']) >= max_speed:
        return 'raw speed limit'
    if abs(data['torque_fb_nm']) >= max_feedback_torque:
        return 'raw torque-feedback limit'
    if data['temperature_c'] >= 60:
        return 'temperature limit'
    return None


def stationary(history):
    """Confirm actual rest: 0.2s LPF speed plus near-constant encoder position.
    Raw hard-speed protection is separate and unchanged.
    """
    if len(history) < 21:
        return False
    if not .18 <= history[-1][0]-history[0][0] <= .30:
        return False
    if any(abs(raw) >= .5 or abs(filtered) >= .15 for _, _, raw, filtered in history):
        return False
    travel = sum(abs((b[1]-a[1]+12.5)%25.-12.5) for a,b in zip(history,list(history)[1:]))
    return travel < .02


def analyze(folder, signal_source="lpf"):
    """Integral regression: int(tau) = J*delta(w) + tc*int(sign w) + b*int(w) + offset*dt.
    Uses transmitted quantized torque, not unverified torque-feedback calibration.
    No acceleration differencing. Both directions needed. Diagnostic candidates only.
    """
    import numpy as np
    folder = Path(folder)
    meta = json.loads((folder/'metadata.json').read_text())
    with (folder/'samples.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    results = {'fit_signal': signal_source, 'simulation': meta['simulation'], 'status': meta.get('status'),
               'primary_model': 'T=J*domega/dt+C*omega',
               'diagnostic_model': 'T=J*domega/dt+tau_c*sign(omega)+b_v*omega+offset',
               'note': 'Candidates only. Command-based torque scale, delays and output-shaft convention must be validated. No robot YAML is modified.',
               'not_measured': ['mass_kg', 'yaw_inertia_kgm2', 'mu_hat', 'brake_damping_Nms'], 'motors': {}}
    for mid in meta['motor_ids']:
        rr = [r for r in rows if int(r['motor_id']) == mid]
        A, y = [], []
        # Non-overlapping ~0.12 s windows; reject loss, reversal, near-zero speed, timing gaps.
        window = []
        for r in rr:
            valid = r['rx_ok'] == '1' and r['phase'] != 'baseline'
            if not valid:
                window = []
                continue
            velocity_key = 'velocity_lpf_rad_s' if signal_source == 'lpf' else 'velocity_rad_s'
            torque_key = 'torque_cmd_lpf_nm' if signal_source == 'lpf' else 'torque_encoded_nm'
            if signal_source == 'lpf' and (not r.get(velocity_key) or not r.get(torque_key) or r.get('filter_reset') == '1'):
                window = []
                continue
            t, w, tx, tq = (float(r[k]) for k in ('rx_time_s', velocity_key, 'tx_time_s', torque_key))
            sign_feature = float(r['velocity_sign_lpf']) if signal_source == 'lpf' else (1. if w > 0 else -1.)
            if abs(w) < .5:
                window = []
                continue
            if window and (int(r['cycle']) != window[-1][4]+1 or t-window[-1][0] > .03 or w*window[-1][1] <= 0):
                window = []
            window.append((t, w, tx, tq, int(r['cycle']), sign_feature))
            if t-window[0][0] < .12:
                continue
            times = np.array([v[0] for v in window]); ws = np.array([v[1] for v in window])
            # Command k acts from tx[k] to tx[k+1]; piecewise constant integral over RX window.
            integ = 0.
            for k, point in enumerate(window):
                left = max(times[0], point[2]); right = min(times[-1], window[k+1][2] if k+1 < len(window) else times[-1])
                integ += max(0., right-left)*point[3]
            dts = np.diff(times)
            A.append([ws[-1]-ws[0], float(np.sum(dts*np.array([v[5] for v in window[:-1]]))),
                      float(np.sum(dts*(ws[:-1]+ws[1:])*.5)), times[-1]-times[0]])
            y.append(integ)
            window = [window[-1]]
        good = [r for r in rr if r['rx_ok'] == '1']
        item = {'rows': len(rr), 'rx_ok': len(good), 'fit_windows': len(A),
                'speed_guard_samples': sum(r.get('speed_guard_active') == '1' for r in rr),
                'peak_speed_rad_s': max([abs(float(r['velocity_rad_s'])) for r in good] or [0.]),
                'peak_command_Nm': max([abs(float(r['torque_encoded_nm'])) for r in rr] or [0.]),
                'max_response_ms': max([float(r['response_ms']) for r in good] or [0.])}
        if len(A) >= 20:
            a = np.array(A); yy = np.array(y); scale = np.linalg.norm(a, axis=0)
            normalized = a/np.maximum(scale, 1e-15)
            coef_scaled, _, rank, _ = np.linalg.lstsq(normalized, yy, rcond=None)
            coef = coef_scaled/np.maximum(scale, 1e-15)
            residual = yy-a@coef
            condition = float(np.linalg.cond(normalized))
            ok = bool(rank == 4 and condition < 100 and coef[0] > 0 and coef[1] >= 0 and coef[2] >= 0 and np.any(a[:,1] > 0) and np.any(a[:,1] < 0))
            simple = a[:, [0, 2]]
            sc = np.linalg.norm(simple, axis=0)
            beta, _, srank, _ = np.linalg.lstsq(simple/np.maximum(sc, 1e-15), yy, rcond=None)
            beta = beta/np.maximum(sc, 1e-15)
            item['primary_model_JC'] = {'wheel_inertia_kgm2': float(beta[0]), 'C_Nms': float(beta[1]),
                'integral_rmse_Nms': float(np.sqrt(np.mean((yy-simple@beta)**2))), 'rank': int(srank),
                'basic_checks_passed': bool(srank == 2 and beta[0] > 0 and beta[1] >= 0), 'needs_review': True}
            fit_speed = item['peak_speed_rad_s']
            item['friction_evaluation_speed_rad_s'] = fit_speed
            item['friction_bound_candidate_Nm'] = float(coef[1] + coef[2]*fit_speed + abs(coef[3])) if ok else None
            item.update(candidate={'wheel_inertia_kgm2': float(coef[0]), 'tau_c_Nm': float(coef[1]),
                                   'b_v_Nms': float(coef[2]), 'torque_offset_Nm': float(coef[3])},
                        integral_rmse_Nms=float(np.sqrt(np.mean(residual**2))), rank=int(rank),
                        normalized_condition=condition, basic_checks_passed=ok,
                        needs_review=True)
        else:
            item['reason'] = 'Insufficient usable windows; do not fill robot parameters from this run.'
        results['motors'][str(mid)] = item
    (folder/'fit_candidates.json').write_text(json.dumps(results, indent=2, allow_nan=False))
    print(json.dumps(results, indent=2, allow_nan=False))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--run', action='store_true', help='operate physical motors')
    mode.add_argument('--simulate', action='store_true', help='synthetic data, no CAN access')
    mode.add_argument('--analyze', metavar='RUN_FOLDER', help='offline candidate fit; requires numpy')
    p.add_argument('--wheels-lifted', action='store_true', help='confirm all wheels clear of ground')
    p.add_argument('--cutoff-hz', type=float, default=5., help='feedback logging LPF cutoff; Hz in (0, 50)')
    p.add_argument('--fit-signal', choices=['raw', 'lpf'], default='lpf', help='offline fit signal; LPF applies to BOTH command torque and velocity')
    p.add_argument('--interface', default='can0')
    p.add_argument('--motors', nargs='+', type=int, choices=WHEELS, default=[1,2,3,4])
    p.add_argument('--max-torque', type=float, default=.10, help='Nm, allowed (0, 0.3]')
    p.add_argument('--max-speed', type=float, default=10., help='raw speed stop rad/s, bench range [2,20]')
    p.add_argument('--max-feedback-torque', type=float, default=.30, help='raw feedback torque stop Nm; <=1.3')
    p.add_argument('--output', type=Path, default=Path.home()/'motor_identification_logs')
    args = p.parse_args()
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    if args.analyze:
        analyze(args.analyze, args.fit_signal)
        return
    if not math.isfinite(args.cutoff_hz) or not 0 < args.cutoff_hz < 50:
        p.error('cutoff-hz must be finite in (0, 50)')
    if not math.isfinite(args.max_torque) or not .01 <= args.max_torque <= .3:
        p.error('max-torque must be finite in [0.01, 0.3] Nm')
    if not math.isfinite(args.max_speed) or not 2 <= args.max_speed <= 20:
        p.error('max-speed must be finite in [2, 20] rad/s')
    if not math.isfinite(args.max_feedback_torque) or not args.max_torque+.005 < args.max_feedback_torque <= SPEC['rated_torque_nm']:
        p.error('max-feedback-torque must exceed command cap +0.005 Nm and be <=1.3 Nm')
    if len(args.motors) != len(set(args.motors)):
        p.error('duplicate motor ID')
    if args.run and not args.wheels_lifted:
        p.error('--run requires --wheels-lifted; this is a suspended-wheel test')
    plan = stages(args.max_torque)
    print('IDs:', {i: WHEELS[i] for i in args.motors}, flush=True)
    print(f'100 Hz, one motor at a time, {sum(s[1] for s in plan):.1f} s/motor. Torque cap {args.max_torque} Nm. Speed stop {args.max_speed} rad/s.', flush=True)
    if not args.run and not args.simulate:
        print('PREVIEW ONLY. Use --simulate for synthetic logs, or --run --wheels-lifted for hardware.')
        return
    folder = args.output/(dt.datetime.now().strftime('%Y%m%d_%H%M%S_%f') + ('_SIMULATED' if args.simulate else '_BENCH'))
    folder.mkdir(parents=True, exist_ok=False)
    meta = {'simulation': args.simulate, 'motor_ids': args.motors, 'mapping': WHEELS,
            'interface': args.interface, 'rate_hz': 100, 'manufacturer_specs': SPEC,
            'command_mode': 'open-loop torque only; no desired speed, PI or velocity brake',
            'max_feedback_torque_Nm': args.max_feedback_torque,
            'current_protection': 'No measured current in this MIT frame. Hardware current protection required; ideal-equivalent current is NOT measured current.',
            'lowpass': {'cutoff_hz': args.cutoff_hz, 'type': 'first-order RC exponential, actual RX dt',
                        'applies_to': ['velocity', 'torque feedback', 'encoded command torque', 'sign(raw velocity)'],
                        'control_and_protection': 'open-loop torque, raw-feedback protection', 'gap_reset_s': .05}, 'max_torque_Nm': args.max_torque,
            'max_speed_rad_s': args.max_speed, 'stages': plan,
            'protocol': {'position': P_RANGE, 'velocity': V_RANGE, 'torque': T_RANGE,
                         'kp': 0, 'kd': 0, 'temperature': 'payload[6]-40; official AK2 MIT manual example',
                         'axis': 'assumed output shaft; verify before fitting'},
            'model_limits': 'Suspended wheels only. Does not identify chassis Iz, mass or ground friction.',
            'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'started_utc': dt.datetime.now(dt.timezone.utc).isoformat(), 'status': 'running'}
    (folder/'metadata.json').write_text(json.dumps(meta, indent=2))
    bus = None; active = None; cycle = 0; sim_time = 0.; start = time.monotonic(); last_flush = start
    try:
        with (folder/'samples.csv').open('w', newline='', buffering=1) as f:
            writer = csv.DictWriter(f, FIELDS); writer.writeheader()
            if args.run:
                bus = CanBus(args.interface, list(WHEELS))
                # This bench script owns all four motor IDs. Stop other motor-control programs.
                for mid in WHEELS:
                    bus.send(mid, DISABLE)
            for mid in args.motors:
                lpf = LowPass(args.cutoff_hz)
                stop_history = deque(maxlen=21)
                active = mid; w = 0.; misses = 0; stopped_count = 0
                if bus:
                    bus.send(mid, pack(0.)[0]); bus.send(mid, ENABLE)
                    time.sleep(.05)
                    bus.drain()
                next_tick = time.monotonic()
                last_tick = None
                for phase, duration, target, control in plan:
                    print(f'{WHEELS[mid]} {phase:8s} {target:+.2f} ({duration:.2f}s)', flush=True)
                    guard_latched = False
                    for k in range(round(duration*100)):
                        if bus:
                            remaining = next_tick-time.monotonic()
                            if remaining > 0:
                                time.sleep(remaining)
                            tick = time.monotonic()
                            late = tick-next_tick
                            if late > .03:
                                raise RuntimeError('Scheduler >30ms late; refusing catch-up burst')
                            step = .01 if last_tick is None else tick-last_tick
                            last_tick = tick
                        else:
                            tick = start+sim_time; late = 0.; step = .01
                        planned = planned_torque(control, target, k*.01, duration)
                        requested = max(-args.max_torque, min(args.max_torque, planned))
                        speed_guard = abs(w) >= .8*args.max_speed and requested*w > 0
                        guard_latched = guard_latched or speed_guard
                        if guard_latched:
                            requested = 0.  # Protection only. No velocity tracking or active braking.
                        payload, actual = pack(requested, args.max_torque)
                        dropped = 0
                        if bus:
                            dropped = bus.drain()
                            tx = time.monotonic(); bus.send(mid, payload)
                            item = bus.receive(mid, tx+.007)
                        else:
                            tx = tick
                            # Known synthetic output-shaft plant: J=.001, tc=.012, b=.003.
                            # Substeps suppress sign chatter at rest.
                            for _ in range(10):
                                drive = actual-.003*w
                                if abs(w) < .003 and abs(drive) <= .012:
                                    w = 0.
                                else:
                                    friction = .012*(1 if (w if abs(w) >= .003 else drive) > 0 else -1)
                                    wn = w + (drive-friction)/.001*.001
                                    w = 0. if w*wn < 0 and abs(drive) < .012 else wn
                            item = ({'position_rad': 0., 'velocity_rad_s': w, 'torque_fb_nm': actual,
                                     'temperature_c': 25., 'error': 0}, tx+.001, '')
                        row = dict.fromkeys(FIELDS, '')
                        row.update(cycle=cycle, motor_id=mid, wheel=WHEELS[mid], phase=phase,
                                   phase_time_s=k*.01, torque_plan_nm=planned,
                                   scheduled_time_s=tick-start-late, tx_time_s=tx-start,
                                   lateness_ms=late*1000, torque_requested_nm=requested,
                                   torque_encoded_nm=actual, rx_ok=int(item is not None), discarded_before_tx=dropped, speed_guard_active=int(guard_latched),
                                   current_ideal_equivalent_a=actual/(SPEC['motor_kt_nm_a']*SPEC['gear_ratio']))
                        if item:
                            data, rx, rawhex = item; w = data['velocity_rad_s']; misses = 0
                            filtered, reset = lpf.update(rx, {
                                'velocity_lpf_rad_s': w, 'torque_fb_lpf_nm': data['torque_fb_nm'],
                                'torque_cmd_lpf_nm': actual,
                                'velocity_sign_lpf': 1. if w > 0 else (-1. if w < 0 else 0.)})
                            row.update(filtered); row['filter_reset'] = int(reset)
                            row.update(data); row.update(rx_time_s=rx-start, unix_rx_s=time.time() if bus else '',
                                                         response_ms=(rx-tx)*1000, payload_hex=rawhex)
                            stop_history.append((rx, data['position_rad'], w, filtered['velocity_lpf_rad_s']))
                        else:
                            misses += 1; stop_history.clear()
                        writer.writerow(row)
                        cycle += 1
                        fault = protection(data, args.max_speed, args.max_feedback_torque) if item else None
                        if fault:
                            raise RuntimeError(f'Motor {mid}: {fault}: {data}')
                        if misses >= 3:
                            raise RuntimeError(f'Motor {mid}: 3 missing responses (~30ms)')
                        if bus and time.monotonic()-last_flush >= 1.:
                            f.flush(); last_flush = time.monotonic()  # fsync must not block the 100 Hz loop
                        sim_time += .01; next_tick += .01
                    if phase in ('baseline', 'stop') and not stationary(stop_history):
                        raise RuntimeError(f'Motor {mid} did not pass LPF speed + encoder rest confirmation for 0.2s. Aborting.')
                if bus:
                    bus.send(mid, pack(0.)[0]); bus.send(mid, DISABLE)
                active = None
            f.flush(); os.fsync(f.fileno())  # all tested motors are disabled
            meta['status'] = 'completed'
    except KeyboardInterrupt:
        meta['status'] = 'interrupted'
    except Exception as exc:
        meta['status'] = 'aborted'; meta['reason'] = str(exc)
        print('STOP:', exc, flush=True)
    finally:
        cleanup = []
        if bus:
            # Independent attempts: one failed send must not skip other motors.
            for mid in WHEELS:
                for payload in (pack(0.)[0], DISABLE, DISABLE):
                    try:
                        bus.send(mid, payload)
                    except Exception as exc:
                        cleanup.append(f'{mid}: {exc}')
            bus.close()
        meta.update(cleanup_errors=cleanup, rows=cycle, finished_utc=dt.datetime.now(dt.timezone.utc).isoformat())
        (folder/'metadata.json').write_text(json.dumps(meta, indent=2))
        print(f"{meta['status']}: {folder}", flush=True)
        print('Zero torque / disable does not guarantee mechanical braking; keep wheels suspended.')
    if meta['status'] == 'aborted':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
