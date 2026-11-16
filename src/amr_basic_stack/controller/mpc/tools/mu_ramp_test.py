#!/usr/bin/python3
"""바닥 마찰계수 μ̂ 토크 램프 시험 (2026-11-16) — 1판 식-21 τ̄ = (r/√2)·μ̂·(mg/4)·c_s 의 μ̂ 를 실측한다.

방법 (한 판)
  1. DAMPING 으로 세운 채 0.8 s 동안 IMU 가속도 평균 = 바이어스.
  2. 방향 키(fwd·back·left·right·fl·fr·bl·br)의 최소 노름 바퀴 토크 패턴을 0 에서 --rate [N·m/s] 로 올린다 (최대 --max).
  3. 매 10 ms: 바퀴로 계산한 몸체 속도 v_w = pinv(T⁻¹)·ω 와 IMU 를 적분한 몸체 속도 v_i 를 그 방향으로 비교.
     v_w − v_i > --slip + 0.15·|v_w| 가 50 ms 이어지면 슬립 판정 → 그 토크를 --hold 초 유지 (바퀴가 헛도는 동안)
     → 토크 0 (0.3 s 굴러가게) → DAMPING 1.5 s. 슬립 없이 --max 에 닿아 0.3 s 지나면 "슬립 없음" (μ̂ 하한).
     |v_w| > --vmax 면 안전 정지 (μ̂ 하한).
  4. 유지 구간에서 v_w − v_i 가 0.05 m/s 넘게 더 벌어져야 "슬립 확인" (아니면 IMU 적분 오차로 본 거짓 판정 → 버림).
     유지 구간의 v_i 기울기 = 미끄러지는 동안 몸체 가속도 a. 지면이 몸체에 준 힘 = m·a (바퀴 회전 관성·구동계 마찰은 몸체 밖이라
     안 들어간다). 그 힘을 내는 최소 노름 지면 토크 τ_g = pinv((T⁻¹)ᵀ)·[m a; 0], 바퀴 롤러 방향 힘 = √2·|τ_g,i|/r
     (식-21 의 45° 롤러 모델, 시뮬 SlipPlant 와 같은 정의). μ̂ = max_i 롤러 힘 / (m g / 4). 앞·옆이면 √2·a/g, 대각이면 2·a/g.
     미끄러지는 동안의 마찰(운동 마찰)이라 출발 순간 최대값(정지 마찰)보다 작거나 같다 → τ̄ 에 쓰기에는 보수 쪽.
     참고로 판정 직전 0.1 s 기울기로 낸 μ̂_onset 과 등방 마찰 가정 |a|/g 도 낸다.

필요: 모터·바퀴 상태·/imu/base_link — `amrall`(use_mpc 기본 false) 또는 amrsens + amrbase. /wheel_torque_cmd 를 내는 다른 것이 없어야 한다.
  /usr/bin/python3 tools/mu_ramp_test.py --dirs fwd,back --repeat 3 --csv ~/capston/capston/analysis/torque_mpc_real/<날짜>_mu/ramp1
  예상 (μ̂ 0.4, 앞): 램프 2 N·m/s 로 약 0.15 s 뒤 슬립, 그때 속도 0.2~0.4 m/s, 유지 0.3 s 동안 계속 가속, 이동 0.3~0.6 m.
  앞뒤 1 m 이상 비워 둘 것. 유지 중 바퀴가 헛돈다 — 바닥 자국·바퀴 마모 주의.
시뮬 검증: torque_experiment.launch.py plant:=sim use_mpc:=false surface_mu:=0.4 use_rviz:=false 를 띄우고 --no-prompt
  (--no-prompt 는 /sim/ground_truth 가 나올 때만 — 실차에서는 판마다 Enter). 시뮬에서는 같은 구간을 참값 속도로 낸 μ̂ 도 찍는다.
  2026-11-16 결과: 참값 속도로는 설정 μ 0.25 → 0.244~0.247 (방법 자체는 맞다). IMU 로는 판마다 흩어진다 —
  μ 0.25 앞 0.206~0.257, 옆 0.117~0.216 / μ 0.4 앞·옆·대각 0.343~0.459. → 방향마다 5 판 이상의 중앙값을 쓰고,
  앞·뒤를 주 값으로 (옆·대각은 롤러 손실·IMU 잡음이 더 크다).
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.executors
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.torque.model import TorquePlant

_D = 1.0 / math.sqrt(2.0)
DIRS = {"fwd": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0),
        "fl": (_D, _D), "fr": (_D, -_D), "bl": (-_D, _D), "br": (-_D, -_D)}
DT = 0.01


def mu_from_accel(a_vec, p):
    """몸체 가속도 a (몸체 좌표 2D) → (μ̂ 롤러 모델 = max_i √2|τ_g,i|/r / (mg/4), 등방 |a|/g, 바퀴별 μ)."""
    F = np.array([p.mass * a_vec[0], p.mass * a_vec[1], 0.0])
    tau_g = np.linalg.pinv(p.Tinv.T) @ F
    mu_i = math.sqrt(2.0) * np.abs(tau_g) / p.r / (p.mass * p.g / 4.0)
    return float(mu_i.max()), float(np.hypot(*a_vec) / p.g), mu_i


def slope(t, v):
    """최소제곱 기울기 (점이 3 개 미만이면 nan)."""
    if len(t) < 3:
        return float("nan")
    return float(np.polyfit(np.asarray(t) - t[0], v, 1)[0])


PH_RAMP, PH_COAST, PH_DAMP, PH_HOLD = 0, 1, 2, 3


def analyze(rec, u, p, gap_grow=0.05):
    """한 판 기록 → 요약. rec 행 = (t, phase, amp, ax, ay, vix, viy, vwx, vwy).
    phase 0 램프 / 3 슬립 판정 뒤 토크 유지 / 1 굴림 / 2 DAMPING.
    결과 "result": slip (유지 구간에서 바퀴–몸체 차가 gap_grow 넘게 벌어짐 — μ̂ 확정) / unconfirmed (거짓 판정, μ̂ 없음) /
    no_slip (최대 토크·안전 속도까지 슬립 없음 — μ̂ 는 하한)."""
    R = np.array(rec)
    t, ph = R[:, 0], R[:, 1]
    vi = R[:, 5:7] @ u
    vw = R[:, 7:9] @ u
    gap = vw - vi
    hold = ph == PH_HOLD
    ramp = ph == PH_RAMP
    nan = float("nan")
    out = {"mu": nan, "mu_iso": nan, "mu_onset": nan, "mu_wheels": None, "a": nan, "amp_slip": nan, "t_slip": nan}
    if hold.any():
        idx = np.where(hold)[0]
        k0 = idx[0]
        out["t_slip"] = float(t[k0] - t[0])
        out["amp_slip"] = float(R[k0, 2])
        out["v_slip"] = float(vw[k0])
        out["gap_grow"] = float(gap[idx[-1]] - gap[k0])
        on = ramp & (t >= t[k0] - 0.1) & (t < t[k0])
        a_on = slope(t[on], vi[on])
        if np.isfinite(a_on):
            out["mu_onset"] = mu_from_accel(a_on * u, p)[0]
        if len(idx) < 10:                                                # 유지가 안전 속도로 0.1 s 안에 끊김 → 기울기 못 냄
            out["result"] = "unconfirmed"
        elif out["gap_grow"] > gap_grow:
            out["result"] = "slip"
            out["a"] = slope(t[idx], vi[idx])
            out["mu"], out["mu_iso"], out["mu_wheels"] = mu_from_accel(out["a"] * u, p)
            if R.shape[1] >= 11 and np.all(np.isfinite(R[idx, 9:11])):      # 시뮬 참값으로 같은 구간 (도구 검증)
                out["mu_truth"] = mu_from_accel(slope(t[idx], R[idx, 9:11] @ u) * u, p)[0]
        else:
            out["result"] = "unconfirmed"
    else:
        out["result"] = "no_slip"
        last = t[ramp][-1] if ramp.any() else t[-1]
        win = ramp & (t >= last - 0.15)
        out["a"] = slope(t[win], vi[win])
        out["v_slip"] = float(vw[ramp][-1]) if ramp.any() else nan
        out["gap_grow"] = nan
        if np.isfinite(out["a"]):
            out["mu"], out["mu_iso"], out["mu_wheels"] = mu_from_accel(out["a"] * u, p)
    out["dist"] = float(np.sum(vi[:-1] * np.diff(t)))
    return out


class RampNode(Node):
    def __init__(self):
        super().__init__("mu_ramp_test")
        self.pub = self.create_publisher(WheelTorqueCommand, "/wheel_torque_cmd", 10)
        self.lock = threading.Lock()
        self.imu = []          # (t, ax, ay)  몸체 좌표
        self.w = None          # 최근 바퀴 각속도 (4,)
        self.w_t = 0.0
        self.truth_t = 0.0
        self.truth_v = (float("nan"), float("nan"))
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=50)
        self.create_subscription(Imu, "/imu/base_link", self.on_imu, q)
        self.create_subscription(WheelStates, "/wheel_states", self.on_wheels, 50)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_truth, 10)

    def on_imu(self, m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9          # 적분 간격은 측정 시각으로 (받은 시각은 몰려 온다)
        with self.lock:
            self.imu.append((time.time(), m.linear_acceleration.x, m.linear_acceleration.y, st))
            if len(self.imu) > 4000:
                del self.imu[:2000]

    def on_wheels(self, m):
        with self.lock:
            self.w = np.array(m.velocity, float)
            self.w_t = time.time()

    def on_truth(self, m):
        """시뮬 전용 참값 — 도구 검증용으로 몸체 속도를 같이 남긴다 (실차에는 없다)."""
        with self.lock:
            self.truth_t = time.time()
            self.truth_v = (m.twist.twist.linear.x, m.twist.twist.linear.y)

    def send(self, mode, tau):
        msg = WheelTorqueCommand()
        msg.mode = mode
        msg.torque = [float(x) for x in tau]
        self.pub.publish(msg)

    def hold(self, mode, tau, sec):
        end = time.time() + sec
        while time.time() < end:
            self.send(mode, tau)
            time.sleep(1.0 / 50.0)

    def imu_since(self, t0):
        with self.lock:
            return [x for x in self.imu if x[0] > t0]


def run_one(node, key, a, p, fk):
    """한 판. 반환 (기록, 이유)."""
    u = np.array(DIRS[key])
    pat = np.linalg.pinv(p.Tinv.T) @ np.array([u[0], u[1], 0.0])
    pat = pat / np.max(np.abs(pat))                      # 최대 바퀴 = 1
    node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.5)
    tb = time.time()
    node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.8)
    with node.lock:
        w = None if node.w is None else node.w.copy()
    bias_s = node.imu_since(tb)
    if w is None or time.time() - node.w_t > 0.3 or len(bias_s) < 20:
        return None, f"실행 안 함: 바퀴 상태 {'없음' if w is None or time.time() - node.w_t > 0.3 else 'OK'}, IMU {len(bias_s)} 개 (0.8 s)"
    if np.max(np.abs(w)) > 0.3:
        return None, f"실행 안 함: 바퀴가 아직 돈다 {np.round(w, 2)} rad/s"
    bias = np.mean([[x[1], x[2]] for x in bias_s], axis=0)
    if not all(np.isfinite(x[3]) and x[3] > 0 for x in bias_s[-5:]):
        return None, "실행 안 함: IMU 메시지에 측정 시각(header.stamp)이 없다"
    rec = []
    v_i = np.zeros(2)
    t_last_imu = time.time()
    st_last = bias_s[-1][3]
    t0 = time.time()
    phase, amp, reason, n_over, t_max_hit, t_phase = PH_RAMP, 0.0, "", 0, None, None
    while True:
        now = time.time()
        el = now - t0
        if phase == PH_RAMP:
            amp = min(a.rate * el, a.max)
            node.send(WheelTorqueCommand.MODE_TORQUE, pat * amp)
        elif phase == PH_HOLD:                                       # 슬립 판정 뒤 같은 토크 유지 (헛도는 동안 몸체 가속 측정)
            node.send(WheelTorqueCommand.MODE_TORQUE, pat * amp)
        elif phase == PH_COAST:
            node.send(WheelTorqueCommand.MODE_TORQUE, [0, 0, 0, 0])
        else:
            node.send(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0])
        for (ti, ax, ay, st) in node.imu_since(t_last_imu):       # IMU 적분 (사각형, 측정 시각 간격, 200 Hz)
            v_i += (np.array([ax, ay]) - bias) * max(0.0, min(st - st_last, 0.02))
            t_last_imu, st_last = ti, st
        with node.lock:
            w = node.w.copy()
            acc = node.imu[-1][1:3] if node.imu else (0.0, 0.0)
            tv = node.truth_v
        v_w = (fk @ w)[:2]
        rec.append((now, phase, amp, acc[0] - bias[0], acc[1] - bias[1], v_i[0], v_i[1], v_w[0], v_w[1], tv[0], tv[1]))
        vu = abs(float(v_w @ u))
        if phase == PH_RAMP:
            d = float((v_w - v_i) @ u)
            n_over = n_over + 1 if d > a.slip + 0.15 * vu else 0
            if n_over >= 5:
                phase, t_phase, reason = PH_HOLD, now, f"슬립 판정 ({amp:.3f} N·m)"
            elif vu > a.vmax:
                phase, t_phase, reason = PH_COAST, now, f"안전 속도 {a.vmax} m/s 넘음 — 슬립 없이 멈춤"
            elif amp >= a.max:
                t_max_hit = t_max_hit or now
                if now - t_max_hit > 0.3:
                    phase, t_phase, reason = PH_COAST, now, f"슬립 없이 최대 {a.max} N·m"
        elif phase == PH_HOLD and (now - t_phase > a.hold or vu > a.vmax):
            phase, t_phase = PH_COAST, now
        elif phase == PH_COAST and now - t_phase > 0.3:
            phase, t_phase = PH_DAMP, now
        elif phase == PH_DAMP and now - t_phase > 1.5:
            break
        time.sleep(DT)
    return rec, reason


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dirs", default="fwd,back", help="방향 키 쉼표로: " + ",".join(DIRS))
    ap.add_argument("--repeat", type=int, default=3, help="방향마다 판 수")
    ap.add_argument("--rate", type=float, default=2.0, help="토크 증가율 [N·m/s] (바퀴 최대값 기준). 빠를수록 낮은 속도에서 슬립")
    ap.add_argument("--max", type=float, default=0.35, help="최대 바퀴 토크 [N·m] (≤ 0.5)")
    ap.add_argument("--slip", type=float, default=0.12, help="슬립 판정 v_w − v_i 문턱 [m/s] (+ 0.15·|v_w|, 50 ms 지속)")
    ap.add_argument("--hold", type=float, default=0.3, help="슬립 판정 뒤 토크 유지 [s] — 이 구간으로 μ̂ 를 잰다 (≤ 0.4)")
    ap.add_argument("--vmax", type=float, default=1.0, help="이 속도를 넘으면 토크를 끊는다 [m/s]")
    ap.add_argument("--csv", default="", help="결과 파일 앞부분. _samples.csv·_summary.csv")
    ap.add_argument("--no-prompt", action="store_true", help="판마다 Enter 없이 (시뮬 전용 — /sim/ground_truth 가 있어야 한다)")
    a = ap.parse_args()
    keys = [k.strip() for k in a.dirs.split(",") if k.strip()]
    if any(k not in DIRS for k in keys):
        sys.exit(f"모르는 방향 — 쓸 수 있는 것: {', '.join(DIRS)}")
    if not (0.0 < a.max <= 0.5 and 0.05 <= a.rate <= 2.0 and 0.1 <= a.vmax <= 1.5 and 0.05 <= a.hold <= 0.4):
        sys.exit("--max ≤ 0.5 N·m, 0.05 ≤ --rate ≤ 2 N·m/s, 0.1 ≤ --vmax ≤ 1.5 m/s, 0.05 ≤ --hold ≤ 0.4 s 로 제한")
    p = TorquePlant.from_yaml()
    fk = np.linalg.pinv(p.Tinv)
    rclpy.init()
    node = RampNode()
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    time.sleep(1.0)
    if a.no_prompt and time.time() - node.truth_t > 1.0:
        sys.exit("--no-prompt 는 시뮬 전용인데 /sim/ground_truth 가 안 나온다 — 실차에서는 판마다 Enter")
    ws = fsum = fsam = None
    if a.csv:
        base = os.path.expanduser(a.csv)
        os.makedirs(os.path.dirname(os.path.abspath(base)) or ".", exist_ok=True)
        fsam = open(base + "_samples.csv", "w", newline="")
        fsum = open(base + "_summary.csv", "w", newline="")
        csv.writer(fsam).writerow(["run", "dir", "t", "phase", "amp", "ax", "ay", "vi_x", "vi_y", "vw_x", "vw_y",
                                   "truth_vx", "truth_vy"])        # truth = 시뮬 참값 (실차는 nan)
        ws = csv.writer(fsum)
        ws.writerow(["run", "dir", "result", "amp_slip", "t_slip", "v_slip", "gap_grow", "a", "mu", "mu_iso", "mu_onset",
                     "dist", "mu_FL", "mu_FR", "mu_RL", "mu_RR"])
    print(f"μ̂ 토크 램프: 방향 {keys} × {a.repeat} 판, {a.rate} N·m/s → 최대 {a.max} N·m, 슬립 문턱 {a.slip} m/s, "
          f"유지 {a.hold} s, m {p.mass:.2f} kg (torque_plant.yaml 질량 + 적재)")
    res = {k: [] for k in keys}
    run = 0
    try:
        for k in keys:
            for _ in range(max(1, a.repeat)):
                run += 1
                if not a.no_prompt:
                    s = input(f"\n=== [{run}] {k}: 앞뒤 1 m 이상 비었는지 확인 후 Enter = 실행, s = 건너뜀, q = 끝: ").strip().lower()
                    if s == "q":
                        raise KeyboardInterrupt
                    if s == "s":
                        continue
                rec, reason = run_one(node, k, a, p, fk)
                if rec is None:
                    print(f"    ★ {reason}")
                    continue
                o = analyze(rec, np.array(DIRS[k]), p)
                res[k].append(o)
                if o["result"] == "slip":
                    txt = (f"μ̂ (롤러 모델) = {o['mu']:.3f}  (등방 |a|/g {o['mu_iso']:.3f}, 판정 직전 {o['mu_onset']:.3f})"
                           f"   유지 중 몸체 가속 {o['a']:.2f} m/s², 바퀴–몸체 차 +{o['gap_grow']:.2f} m/s"
                           + (f"   [시뮬 참값 속도로 {o['mu_truth']:.3f}]" if "mu_truth" in o else ""))
                elif o["result"] == "unconfirmed":
                    txt = (f"★ 슬립 확인 안 됨 (유지 중 바퀴–몸체 차 {o['gap_grow']:+.2f} m/s, 또는 유지가 0.1 s 안에 안전 속도로 끊김)"
                           f" — 이 판은 버림")
                else:
                    txt = f"μ̂ ≥ {o['mu']:.3f} (하한, 마지막 0.15 s 몸체 가속 {o['a']:.2f} m/s²)"
                print(f"    [{run}] {k}: {reason}.  속도 {o['v_slip']:.2f} m/s, 이동 {o['dist']:.2f} m\n         {txt}")
                if fsam:
                    wr = csv.writer(fsam)
                    for row in rec:
                        wr.writerow([run, k, f"{row[0] - rec[0][0]:.3f}", *[f"{x:.5f}" for x in row[1:]]])
                    mw = o["mu_wheels"] if o["mu_wheels"] is not None else [float("nan")] * 4
                    ws.writerow([run, k, o["result"], o["amp_slip"], o["t_slip"], o["v_slip"], o["gap_grow"], o["a"],
                                 o["mu"], o["mu_iso"], o["mu_onset"], o["dist"], *mw])
                    fsam.flush()
                    fsum.flush()
    except KeyboardInterrupt:
        print("\n중단")
    finally:
        node.hold(WheelTorqueCommand.MODE_DAMPING, [0, 0, 0, 0], 0.5)
        print("DAMPING 보냄.")
        print("\n요약 (슬립 확인된 판만 μ̂, 슬립 없는 판은 하한, 거짓 판정은 수만)")
        for k, lst in res.items():
            sl = [o["mu"] for o in lst if o["result"] == "slip" and np.isfinite(o["mu"])]
            lo = [o["mu"] for o in lst if o["result"] == "no_slip" and np.isfinite(o["mu"])]
            un = sum(o["result"] == "unconfirmed" for o in lst)
            print(f"  {k:5s}: μ̂ {('중앙 %.3f (%s)' % (np.median(sl), ', '.join('%.3f' % x for x in sl))) if sl else '—'}"
                  f"{('   하한 ≥ %.3f' % max(lo)) if lo else ''}{('   거짓 판정 %d 판' % un) if un else ''}")
        for f in (fsam, fsum):
            if f:
                f.close()
        ex.shutdown()
        th.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
