#!/usr/bin/python3
"""AK40-10 모터 하나 벤치 시험 (CANable + 모터 1개, 바퀴를 띄우거나 모터만) — To do 1절 측정용. ROS 없이 돈다.

  ★ 모터는 상위 프로그램(CubeMars Tool, 시리얼 CP2102)으로 먼저 MIT 모드로 바꾸고 CAN ID 를 정해 둔다 (매뉴얼 4.1.1.4, 4.1.1.10).
     공장 기본 ID 는 1. 벤치는 모터 하나씩이라 ID 가 겹쳐도 되지만, 차에 달기 전에는 1~4 로 나눠야 한다.
  ★ 바퀴는 바닥에서 띄운다. 모든 명령은 --max_tau(기본 0.5 N·m) 와 --time(최대 10 s) 안에서만 낸다.
     damping 의 spin(속도 명령) 도 kd·|v_des − v| ≤ max_tau 가 되게 v_des 를 한 주기씩 올린다 (2026-11-14 전에는 kd 0.5 × 20 rad/s
     = 10 N·m 를 요구했다 — 응답으로 본 최대는 0.4 N·m).
     Ctrl+C 를 누르면 DAMPING 을 0.5 s 보낸 뒤 모터 모드에서 나온다.

  CAN 켜기 (candleLight 펌웨어):  sudo ip link set can0 up type can bitrate 1000000
  /usr/bin/python3 tools/ak40_bench.py status --id 1                     # 응답 확인 (위치·속도·토크·온도·에러)
  /usr/bin/python3 tools/ak40_bench.py torque --id 1 --tau 0.05 --time 2  # 일정 토크 → 가속(J_w), 끝속도
  /usr/bin/python3 tools/ak40_bench.py steps --id 1 --taus 0.02,0.04,0.06,0.1 --hold 3   # 토크 계단 → 정상 속도 = τ_fric(ω)
  /usr/bin/python3 tools/ak40_bench.py damping --id 1 --spin 20           # 20 rad/s 로 돌린 뒤 DAMPING → 서는 시간
  --csv 경로 를 주면 [t, mode, p, v, τ_응답, 온도, 에러] 를 남긴다.

무엇을 재나 (To do 1절)
  torque  : 처음 0.2 s 속도 기울기 α 로 J_w ≈ τ/α (마찰을 빼면 τ − τ_fric). 끝속도 = 그 토크의 정상 속도
  steps   : 각 계단 마지막 1 s 평균 속도 → (ω, τ) 점들 = 구동계 마찰 곡선. 응답 토크와 명령 토크 차이는 12 비트 양자화만
            보여 준다 — 드라이버는 명령 토크를 Iq = τ/K_T 로 바꾸고 응답 토크도 측정 전류 × K_T 로 내므로 같은 K_T 가정을
            양쪽에 쓴다. 바퀴에 실제로 나간 토크의 정확도는 관성을 아는 가속 시험(τ ≈ J·α + τ_fric)이나 외부 저울로 잰다
  damping : kd = torque_plant.yaml damping_kd 로 서는 데 걸린 시간. 무부하 감쇠 시정수 ≈ J_w/kd
"""
import argparse
import csv
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from amr_mpc.ak40 import protocol as P   # noqa: E402
from amr_mpc.ak40.bus import CanBus      # noqa: E402
from amr_mpc.core import config          # noqa: E402


class Bench:
    def __init__(self, a):
        ak = config.load("ak40.yaml")["ak40"]
        if not ak["manual_checked"] and not a.unverified_ok:
            raise SystemExit("config/ak40.yaml manual_checked: false — mit_limits 를 매뉴얼 값으로 고친 뒤 돌린다")
        self.lim = P.MitLimits.from_dict(ak["mit_limits"])
        rid = str(ak.get("reply_can_id", "motor")).strip().lower()
        self.reply_id = None if rid == "motor" else int(rid)
        self.temp_offset = int(ak["temp_offset"])
        self.id, self.max_tau, self.period = a.id, a.max_tau, 1.0 / a.rate
        self.kd = float(config.load("torque_plant.yaml")["motor"]["damping_kd"])
        self.bus = CanBus(a.interface)
        self.rows, self.t0 = [], time.monotonic()
        self.csv = a.csv

    def send(self, data):
        self.bus.send(self.id, data)
        t_end = time.monotonic() + 0.005
        while time.monotonic() < t_end:
            f = self.bus.recv(t_end - time.monotonic())
            if f and len(f[1]) >= 6 and f[1][0] == self.id and f[0] == (self.id if self.reply_id is None else self.reply_id):
                return P.unpack_reply(self.lim, f[1], self.temp_offset)
        return None

    def cmd(self, mode, p=0.0, v=0.0, kp=0.0, kd=0.0, t=0.0):
        t = float(np.clip(t, -self.max_tau, self.max_tau))
        r = self.send(P.pack_command(self.lim, p, v, kp, kd, t))
        if r is not None:
            self.rows.append([time.monotonic() - self.t0, mode, r["p"], r["v"], r["t"], r["temp"], r["error"]])
        return r

    def run(self, mode, seconds, **kw):
        """seconds 동안 rate 로 같은 명령. 응답 목록."""
        out, t_end = [], time.monotonic() + min(seconds, 10.0)
        while time.monotonic() < t_end:
            t_next = time.monotonic() + self.period
            r = self.cmd(mode, **kw)
            if r is not None:
                out.append((time.monotonic() - self.t0, r))
            time.sleep(max(0.0, t_next - time.monotonic()))
        return out

    def spin(self, v_target, seconds, kd):
        """속도 명령으로 돌린다. 모터 쪽 토크 kd·(v_des − v) 가 max_tau 를 넘지 않게 v_des 를 마지막 응답 속도 ± max_tau/kd 로 자른다."""
        out, v_now, step = [], 0.0, self.max_tau / kd
        t_end = time.monotonic() + min(seconds, 10.0)
        while time.monotonic() < t_end:
            t_next = time.monotonic() + self.period
            v_des = v_now + float(np.clip(v_target - v_now, -step, step))
            r = self.cmd("spin", v=v_des, kd=kd)
            if r is not None:
                v_now = r["v"]
                out.append((time.monotonic() - self.t0, r))
            time.sleep(max(0.0, t_next - time.monotonic()))
        return out

    def enter(self):
        r = self.send(P.ENTER_MOTOR_MODE)
        print(f"모터 {self.id} 모드 진입: {'응답 ' + str(_fmt(r)) if r else '응답 없음 (ID·배선·비트레이트 확인)'}")
        self.run("damping", 0.3, kd=self.kd)

    def close(self):
        self.run("damping", 0.5, kd=self.kd)
        self.send(P.EXIT_MOTOR_MODE)
        self.bus.close()
        if self.csv and self.rows:
            with open(self.csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "mode", "p", "v", "tau_reply", "temp", "error"])
                w.writerows(self.rows)
            print(f"저장: {len(self.rows)}행 → {self.csv}")


def _fmt(r):
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()} if r else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["status", "torque", "steps", "damping"])
    ap.add_argument("--id", type=int, required=True)
    ap.add_argument("--interface", default="can0")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--max_tau", type=float, default=0.5)
    ap.add_argument("--tau", type=float, default=0.05)
    ap.add_argument("--time", type=float, default=2.0)
    ap.add_argument("--taus", default="0.02,0.04,0.06,0.1")
    ap.add_argument("--hold", type=float, default=3.0)
    ap.add_argument("--spin", type=float, default=20.0)
    ap.add_argument("--csv", default="")
    ap.add_argument("--unverified-ok", dest="unverified_ok", action="store_true")
    a = ap.parse_args()
    b = Bench(a)
    try:
        b.enter()
        if a.what == "status":
            for _, r in b.run("damping", 1.0, kd=b.kd)[-3:]:
                print(_fmt(r))
        elif a.what == "torque":
            rs = b.run("torque", a.time, t=a.tau)
            t = np.array([x[0] for x in rs]); v = np.array([x[1]["v"] for x in rs])
            m = t < t[0] + 0.2
            alpha = np.polyfit(t[m], v[m], 1)[0] if m.sum() > 3 else math.nan
            print(f"토크 {a.tau} N·m: 처음 0.2 s 가속 {alpha:.2f} rad/s² → J_w ≈ {a.tau/alpha if alpha else math.nan:.2e} kg·m² "
                  f"(마찰 포함, 위쪽 추정)   끝속도 {v[-10:].mean():.2f} rad/s   응답 토크 평균 {np.mean([x[1]['t'] for x in rs[-50:]]):.4f}")
        elif a.what == "steps":
            print("명령 τ [N·m] | 정상 속도 ω [rad/s] | 응답 τ 평균 | 차이")
            for tau in [float(x) for x in a.taus.split(",")]:
                rs = b.run("torque", a.hold, t=tau)
                tail = [x[1] for x in rs if x[0] > rs[-1][0] - 1.0]
                w = np.mean([r["v"] for r in tail]); tr = np.mean([r["t"] for r in tail])
                print(f"{tau:8.3f} | {w:8.2f} | {tr:8.4f} | {tr - tau:+.4f}")
        elif a.what == "damping":
            b.spin(a.spin, 1.5, kd=0.5)
            rs = b.run("damping", 3.0, kd=b.kd)
            v0 = rs[0][1]["v"]
            stop = next((x[0] - rs[0][0] for x in rs if abs(x[1]["v"]) < 0.05 * max(abs(v0), 1e-6)), math.nan)
            print(f"DAMPING kd {b.kd}: {v0:.1f} rad/s 에서 5 % 아래까지 {stop:.3f} s (시정수 ≈ {stop/3:.3f} s → J ≈ kd·τ = {b.kd*stop/3:.2e})")
    except KeyboardInterrupt:
        print("중단 — DAMPING 뒤 모터 모드 해제")
    finally:
        b.close()


if __name__ == "__main__":
    main()
