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
  /usr/bin/python3 tools/ak40_bench.py steps --id 1 --taus 0.1,0.05,0.08,0.05,0.08,0.05,0.08,0.05,0.08,0 --hold 1.0 --csv jw_1_1.csv
                                                                          # J_w: 돌고 있는 중 토크 계단 → tools/jw_step_analysis.py
  --csv 경로 를 주면 [t, mode, p, v, τ_응답, 온도, 에러, τ_명령] 을 남긴다 (τ_명령 열은 2026-11-16 부터).

  lifted (2026-11-24, 바퀴 회전자 식별 지시서 W1 — 바퀴를 띄운 토크 단계 시험. 위 네 명령의 동작은 그대로다)
    /usr/bin/python3 tools/ak40_bench.py lifted --id 2 --taus 0.08,0.045,0.03,0.04,0.035,0.045,0 --holds 0.5,4,4,4,4,4,1.5 \
        --out_dir analysis/torque_mpc_real/2026-11-24_lifted --tag fwd_r1
    음 방향은 --taus=-0.08,-0.045,…  (값이 - 로 시작하면 = 로 붙여 쓴다). 승인된 순서는 tools/ak40_lifted_w1.sh 가 대신 넣어 준다.
    steps 와 다른 점:
      · 단계마다 유지 시간 (--holds). 첫 단계는 정지 마찰을 넘기는 출발용 — 끝났는데 |ω| < --start_min 이면 중단
      · 부호가 바퀴 기준 (--frame wheel 기본: + = 전진. config/ak40.yaml direction 으로 모터 부호를 바꿔 보내고, 기록도 바퀴 기준)
      · |ω| > --v_max (기본 = torque_plant.yaml 정격 속도 38.7 rad/s) 면 토크 0 으로 바꾸고 끝낸다 (속도 상한)
      · 응답이 --max_miss 주기 연속 없거나 모터 에러 코드가 0 이 아니면 끝낸다
      · |τ| > --tau_cap (기본 0.1 N·m) 인 단계가 있으면 시작하지 않는다 (자르지 않고 거부). 단계 ≤ 10 s, 합 ≤ 60 s
      · CSV 에 실제로 나간 값 tau_cmd_q (12 비트 반올림 뒤, "0" 은 ±1.22 mN·m) 와 단계 번호 step 열이 더 있다
      · 같은 이름의 .json 에 조건·시작/끝 온도·git 커밋·단계별 끝 1 s 평균을 남긴다

무엇을 재나 (To do 1절)
  torque  : 처음 0.2 s 속도 기울기 α 로 J_w ≈ τ/α (마찰을 빼면 τ − τ_fric). 끝속도 = 그 토크의 정상 속도
            ※ 마찰 모델에 따라 J 가 6e-4 ~ 1.0e-3 로 흩어졌다 (2026-11-16). J_w 는 아래 steps 계단 차분으로 잰다:
              계단 순간 앞뒤 ω̇ 차 → J = Δτ/Δω̇ (같은 ω 라 마찰이 소거된다, tools/jw_step_analysis.py)
  steps   : 각 계단 마지막 1 s 평균 속도 → (ω, τ) 점들 = 구동계 마찰 곡선. 응답 토크와 명령 토크 차이는 12 비트 양자화만
            보여 준다 — 드라이버는 명령 토크를 Iq = τ/K_T 로 바꾸고 응답 토크도 측정 전류 × K_T 로 내므로 같은 K_T 가정을
            양쪽에 쓴다. 바퀴에 실제로 나간 토크의 정확도는 관성을 아는 가속 시험(τ ≈ J·α + τ_fric)이나 외부 저울로 잰다
  damping : kd = torque_plant.yaml damping_kd 로 서는 데 걸린 시간. 무부하 감쇠 시정수 ≈ J_w/kd
"""
import argparse
import csv
import json
import math
import os
import subprocess
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
        if r is not None:                              # tau_cmd = MIT 명령의 토크 항 t (2026-11-16, J_w 계단 분석용)
            self.rows.append([time.monotonic() - self.t0, mode, r["p"], r["v"], r["t"], r["temp"], r["error"], t])
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
                w.writerow(["t", "mode", "p", "v", "tau_reply", "temp", "error", "tau_cmd"])
                w.writerows(self.rows)
            print(f"저장: {len(self.rows)}행 → {self.csv}")


# ── 2026-11-24 바퀴를 띄운 토크 단계 시험 (lifted, 바퀴 회전자 식별 지시서 W1) ─────────────────────────
#    위 Bench.cmd·run (status·torque·steps·damping 이 쓰는 길) 은 건드리지 않는다. lifted 는 Bench.send 만 쓴다.
WHEELS = ("FL", "FR", "RL", "RR")
LIFTED_COLUMNS = ["t", "mode", "p", "v", "tau_reply", "temp", "error", "tau_cmd", "tau_cmd_q", "step"]


def quantize_torque(lim, tau_motor):
    """MIT 12 비트로 실제 나가는 모터 토크 (protocol.pack_command 와 같은 반올림)."""
    return P.uint_to_float(P.float_to_uint(tau_motor, -lim.t_max, lim.t_max, 12), -lim.t_max, lim.t_max, 12)


def lifted_plan(taus, holds, sign, lim, tau_cap, max_hold=10.0, max_total=60.0):
    """단계표. taus 는 기록 좌표 (sign = ±1: 그 좌표의 + 가 모터의 ± — 바퀴 기준이면 ak40.yaml direction, 모터 기준이면 +1).
    반환 [dict(step, tau, hold, tau_motor, tau_q)] — tau_q 는 실제 나가는 값을 다시 기록 좌표로 돌린 것.
    조건이 안 맞으면 ValueError (자르지 않고 거부한다)."""
    taus, holds = [float(x) for x in taus], [float(x) for x in holds]
    if not taus or len(taus) != len(holds):
        raise ValueError(f"taus {len(taus)} 개 · holds {len(holds)} 개 — 같은 수로 준다")
    if sign not in (1, -1, 1.0, -1.0):
        raise ValueError(f"sign {sign} — ±1")
    if not (0.0 < tau_cap <= lim.t_max):
        raise ValueError(f"tau_cap {tau_cap} — 0 < tau_cap ≤ {lim.t_max}")
    for k, (tau, hold) in enumerate(zip(taus, holds)):
        if not math.isfinite(tau) or abs(tau) > tau_cap + 1e-12:
            raise ValueError(f"단계 {k}: |τ| {abs(tau)} > tau_cap {tau_cap} N·m — 시작하지 않는다")
        if not (0.0 < hold <= max_hold):
            raise ValueError(f"단계 {k}: hold {hold} s — 0 < hold ≤ {max_hold}")
    if sum(holds) > max_total:
        raise ValueError(f"유지 시간 합 {sum(holds)} s > {max_total} s")
    plan = []
    for k, (tau, hold) in enumerate(zip(taus, holds)):
        tm = sign * tau
        plan.append({"step": k, "tau": tau, "hold": hold, "tau_motor": tm, "tau_q": sign * quantize_torque(lim, tm)})
    return plan


def run_lifted(io, plan, sign, lim, period, v_max, start_min=3.0, max_miss=10, trip_coast=1.0,
               clock=time.monotonic, sleep=time.sleep):
    """단계표를 한 주기씩 내보낸다. io.send(8 바이트) → 응답 dict (모터 기준 p·v·t, temp, error) 또는 None.
    반환 (rows, info). rows 는 LIFTED_COLUMNS 순서 (기록 좌표 = sign 을 곱한 값), info 는 끝난 이유·누락 수·온도.
    끝내는 조건: 단계표를 다 냄 / |ω| > v_max (토크 0 으로 trip_coast 초 더 기록) / 첫 단계가 끝났는데 |ω| < start_min /
                 응답이 max_miss 주기 연속 없음 / 모터 에러 코드."""
    rows, info = [], {"stop": "완료", "missed": 0, "temp_start": math.nan, "temp_end": math.nan, "v_peak": 0.0}
    t0 = clock()
    miss_run, last_v = 0, 0.0

    def tick(mode, step, tau, tau_m, tau_q):
        nonlocal miss_run, last_v
        t_next = clock() + period
        r = io.send(P.pack_command(lim, 0.0, 0.0, 0.0, 0.0, tau_m))
        stop = None
        if r is None:
            info["missed"] += 1
            miss_run += 1
            if miss_run >= max_miss:
                stop = f"응답 없음 {miss_run} 주기 연속"
        else:
            miss_run = 0
            last_v = sign * r["v"]
            info["v_peak"] = max(info["v_peak"], abs(last_v))
            if math.isnan(info["temp_start"]):
                info["temp_start"] = r["temp"]
            info["temp_end"] = r["temp"]
            rows.append([clock() - t0, mode, sign * r["p"], last_v, sign * r["t"], r["temp"], r["error"], tau, tau_q, step])
            if r["error"] != 0:
                stop = f"모터 에러 {r['error']} ({P.ERROR_NAMES.get(r['error'], '알 수 없음')})"
            elif abs(last_v) > v_max:
                stop = f"속도 상한: |ω| {abs(last_v):.1f} > {v_max:.1f} rad/s"
        sleep(max(0.0, t_next - clock()))
        return stop

    stop = None
    for st in plan:
        t_end = clock() + st["hold"]
        while clock() < t_end and stop is None:
            stop = tick("torque", st["step"], st["tau"], st["tau_motor"], st["tau_q"])
        if stop is None and st["step"] == plan[0]["step"] and start_min > 0.0 and abs(last_v) < start_min:
            stop = f"출발 실패: 첫 단계 뒤 |ω| {abs(last_v):.2f} < {start_min} rad/s (정지 마찰을 못 넘음)"
        if stop is not None:
            break
    if stop is not None:
        info["stop"] = stop
        zero_q = sign * quantize_torque(lim, 0.0)
        t_end = clock() + trip_coast                    # 토크 0 으로 바꿔 두고 조금 더 기록한다 (뒤에 Bench.close 가 DAMPING)
        while clock() < t_end:
            tick("trip", -1, 0.0, 0.0, zero_q)
    return rows, info


def lifted_step_summary(rows, plan, tail=1.0):
    """단계별 끝 tail 초 평균 [dict(step, tau, tau_q, hold, n, w_end, tau_reply_end)] — 표본이 없는 단계는 n 0."""
    out = []
    for st in plan:
        r = [x for x in rows if x[9] == st["step"] and x[1] == "torque"]
        rec = {"step": st["step"], "tau": st["tau"], "tau_q": st["tau_q"], "hold": st["hold"], "n": len(r),
               "w_end": math.nan, "tau_reply_end": math.nan}
        if r:
            sel = [x for x in r if x[0] > r[-1][0] - tail]
            rec["w_end"] = float(np.mean([x[3] for x in sel]))
            rec["tau_reply_end"] = float(np.mean([x[4] for x in sel]))
        out.append(rec)
    return out


def _git_state():
    """(커밋 해시, 커밋 안 된 변경 수) — git 이 없거나 실패하면 ("", -1)."""
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        head = subprocess.run(["git", "-C", here, "rev-parse", "HEAD"], capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(["git", "-C", here, "status", "--porcelain"], capture_output=True, text=True, timeout=5).stdout
        return head, len([x for x in dirty.splitlines() if x.strip()])
    except Exception:
        return "", -1


def lifted_prepare(a):
    """lifted 인자 → 단계표·부호·속도 상한. CAN 을 열기 전에 부른다 (조건이 안 맞으면 SystemExit — 모터 모드에 들어가지 않는다)."""
    ak = config.load("ak40.yaml")["ak40"]
    plant = config.load("torque_plant.yaml")
    lim = P.MitLimits.from_dict(ak["mit_limits"])
    ids = [int(i) for i in ak["can_ids"]]
    if a.id not in ids:
        raise SystemExit(f"--id {a.id} 가 config/ak40.yaml can_ids {ids} 에 없다 — 바퀴 기준 부호를 정할 수 없다")
    wi = ids.index(a.id)
    direction = int(ak["direction"][wi])
    sign = float(direction) if a.frame == "wheel" else 1.0
    v_max = a.v_max if a.v_max > 0.0 else float(plant["motor"]["rated_speed_rpm"]) * 2.0 * math.pi / 60.0
    if not (v_max < lim.v_max):
        raise SystemExit(f"lifted: v_max {v_max} 가 MIT 속도 범위 ±{lim.v_max} rad/s 이상 — 응답 속도가 거기서 잘려 상한을 볼 수 없다")
    try:
        taus = [float(x) for x in a.taus.split(",")]
        holds = [float(x) for x in a.holds.split(",")] if a.holds else [a.hold] * len(taus)
        plan = lifted_plan(taus, holds, sign, lim, a.tau_cap)
    except ValueError as e:
        raise SystemExit(f"lifted: {e}")
    return {"plan": plan, "sign": sign, "v_max": v_max, "wheel": wi, "direction": direction, "holds": holds, "plant": plant}


def lifted_main(b, a, prep, **run_kw):
    """lifted 명령 — Bench b (CAN 연결·모터 모드 진입은 lifted_cli 가 한다), 인자 a, lifted_prepare 결과.
    run_kw 는 run_lifted 로 넘긴다 (시험에서 clock·sleep 을 갈아 끼운다)."""
    plan, sign, v_max, wi, direction, holds, plant = (prep[k] for k in ("plan", "sign", "v_max", "wheel", "direction", "holds", "plant"))
    print(f"lifted: 모터 {a.id} = {WHEELS[wi]} (장착 부호 {direction:+d}), 기록 좌표 {a.frame} "
          f"({'+ = 전진' if a.frame == 'wheel' else '+ = 모터 +'}), 속도 상한 {v_max:.1f} rad/s, 토크 한도 {a.tau_cap} N·m, "
          f"합 {sum(holds):.1f} s")
    print("  단계 | 명령 τ | 실제 나가는 τ | 유지 s")
    for st in plan:
        print(f"  {st['step']:4d} | {st['tau']:+.4f} | {st['tau_q']:+.5f} | {st['hold']:.1f}")
    rows, info = run_lifted(b, plan, sign, b.lim, b.period, v_max, a.start_min, a.max_miss, **run_kw)
    summ = lifted_step_summary(rows, plan)
    print(f"끝: {info['stop']}.  표본 {len(rows)}, 응답 누락 {info['missed']}, |ω| 최대 {info['v_peak']:.1f} rad/s, "
          f"온도 {info['temp_start']:.0f} → {info['temp_end']:.0f} °C")
    print("  단계 | 명령 τ | 끝 1 s 평균 ω [rad/s] | 응답 τ 평균")
    for s_ in summ:
        print(f"  {s_['step']:4d} | {s_['tau']:+.4f} | {s_['w_end']:8.2f} | {s_['tau_reply_end']:+.4f}")
    path = a.csv
    if not path and a.out_dir:
        os.makedirs(a.out_dir, exist_ok=True)
        tag = f"_{a.tag}" if a.tag else ""
        path = os.path.join(a.out_dir, f"{time.strftime('%Y%m%d_%H%M')}_lifted_{WHEELS[wi]}{tag}.csv")
        k = 1
        while os.path.exists(path):                     # 같은 분에 두 판이면 덮어쓰지 않는다
            k += 1
            path = os.path.join(a.out_dir, f"{time.strftime('%Y%m%d_%H%M')}_lifted_{WHEELS[wi]}{tag}_{k}.csv")
    if path and rows:
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(LIFTED_COLUMNS)
            w.writerows(rows)
        head, dirty = _git_state()
        meta = {
            "time": time.strftime("%Y-%m-%d %H:%M:%S"), "csv": os.path.basename(path), "columns": LIFTED_COLUMNS,
            "motor_id": a.id, "wheel": WHEELS[wi], "direction": direction, "frame": a.frame, "sign": sign,
            "rate_hz": 1.0 / b.period, "v_max": v_max, "tau_cap": a.tau_cap, "start_min": a.start_min, "max_miss": a.max_miss,
            "plan": plan, "summary": summ, "info": info, "n_rows": len(rows),
            "torque_step_Nm": b.lim.step("t"), "mit_limits": {"t_max": b.lim.t_max, "v_max": b.lim.v_max, "p_max": b.lim.p_max},
            "config": {"J_w": plant["wheel"]["inertia_J_w_kgm2"], "coulomb_Nm": plant["friction"]["coulomb_Nm"],
                       "viscous_Nms_per_rad": plant["friction"]["viscous_Nms_per_rad"]},
            "git_commit": head, "git_dirty_files": dirty,
        }
        with open(os.path.splitext(path)[0] + ".json", "w") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        print(f"저장: {len(rows)}행 → {path}  (+ .json)")
    return info


def _fmt(r):
    return {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()} if r else None


def lifted_cli(a):
    """lifted: 단계표를 먼저 검사하고 (안 맞으면 CAN 을 열지 않는다), CSV 는 lifted_main 이 따로 쓴다."""
    prep = lifted_prepare(a)
    a_csv, a.csv = a.csv, ""            # Bench.close 가 옛 형식 CSV 를 쓰지 않게 비운다
    b = Bench(a)
    a.csv = a_csv
    try:
        b.enter()
        lifted_main(b, a, prep)
    except KeyboardInterrupt:
        print("중단 — DAMPING 뒤 모터 모드 해제 (이 판은 저장하지 않는다)")
    finally:
        b.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["status", "torque", "steps", "damping", "lifted"])
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
    # ── lifted 전용 (2026-11-24). 다른 명령에서는 안 쓰인다
    ap.add_argument("--holds", default="", help="lifted: 단계별 유지 시간 [s] (쉼표, taus 와 같은 수). 비우면 --hold 를 모든 단계에")
    ap.add_argument("--frame", choices=["wheel", "motor"], default="wheel", help="lifted: 토크·기록 부호. wheel = + 가 전진 (ak40.yaml direction)")
    ap.add_argument("--v_max", type=float, default=0.0, help="lifted: 속도 상한 [rad/s]. 0 = torque_plant.yaml 정격 속도")
    ap.add_argument("--tau_cap", type=float, default=0.1, help="lifted: 이보다 큰 |τ| 단계가 있으면 시작하지 않는다 [N·m]")
    ap.add_argument("--start_min", type=float, default=3.0, help="lifted: 첫 단계 뒤 |ω| 가 이보다 작으면 중단 [rad/s] (0 = 검사 안 함)")
    ap.add_argument("--max_miss", type=int, default=10, help="lifted: 응답이 이만큼 연속 없으면 중단 [주기]")
    ap.add_argument("--out_dir", default="", help="lifted: 여기에 YYYYMMDD_HHMM_lifted_<바퀴>_<tag>.csv 로 저장 (--csv 가 우선)")
    ap.add_argument("--tag", default="", help="lifted: 파일 이름 끝 (예 fwd_r1)")
    a = ap.parse_args()
    if a.what == "lifted":
        lifted_cli(a)
        return
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
