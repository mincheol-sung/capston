#!/usr/bin/env python3
"""seq — controller:=all 스택에서 제어기를 번갈아 사람 없이 여러 회차를 돈다 (2026-11-28 사용자 결정).

  amrseq                          지금 기준 (rqt · pit s1|s2|s3 로 맞춘 다음 판) 으로 mpc → pp → adpp 를 N 바퀴 (기본 6 — 2026-11-28 사용자 결정, 세 경로 모두)
  amrseq --n 6                    제어기마다 6 회차
  amrseq --order mpc,adpp         고를 제어기·순서
  amrseq --dry                    할 일만 출력 (무장 안 함)
  amrseq --no-anchor              출발점을 고정하지 않는다 (매 회차 무장한 자리 = 출발점, 예전 origin robot)

먼저: amrall use_mpc:=true controller:=all limit_mode:=friction bag:=… 로 띄우고, 초록 배너 ("측위 맞추기 끝: 맞음") 를 본 뒤 실행.

회차마다
  1. 제어기 고르기 (controller_select_node active) · 출발점 (origin_mode fixed, 아래 anchor) 넣기
  2. 정지 확인: /mpc/armed false, 기준 idle, 로봇 속력 < 0.03 m/s · |ω| < 0.05 rad/s 가 settle 초 동안
  3. /mpc/arm true (패드 RT 와 같은 토픽) → /mpc/armed true · 기준 running 확인
  4. 기준 finished 까지 기다림. end done = 회차 끝 → /mpc/arm false
  zigzag 는 출발점에서 2.5 m 떨어져 끝나므로 회차마다 복귀 다리: 끝 자리에서 제자리 180° 회전 (shape rotate, --return-ctrl 제어기).
    180° 돈 뒤의 zigzag 는 같은 바닥 선을 거꾸로 지나 출발점으로 돌아온다 (계산: 두 경로 거리 최대 0.00 cm, COMMANDS_KO.md).
    단 zigzag_n 이 홀수일 때만 — 짝수면 출발점으로는 돌아오지만 다른 바닥을 지난다 (n 4 · L 2.5: 최대 2.18 m) → 기본 거부 (--allow-even).
    → 회차 방향이 fwd·rev 번갈아. 제어기 셋 × 짝수 N 이면 제어기마다 fwd·rev 가 같은 수.
anchor (기본 켬): 시작할 때 로봇 포즈 A 를 출발점으로 잡아 origin_mode fixed 로 넣는다 (zigzag rev 는 A 에서 경로 끝 B, 방향 +180°).
  회차 끝 위치·헤딩 오차가 다음 회차 출발점으로 쌓이지 않는다. 출발점에서 start_tolerance (30 cm · 30°) 밖이면 기준 노드가 출발을 거부 → 중단.
  진입 구간은 start_delay (3 s) 동안 지금 자리 → 출발점 스무스스텝 (lead_speed 0.10 m/s 이하) 이라 몇 cm 오차면 3 s 안에 맞춘다.

중단 (다시 무장하지 않는다 — 사용자 결정 2026-11-28 "전체 순서 중단"):
  회차 도중 무장이 꺼짐 (fault_monitor · 's' · 패드 B) · 기준 abort (abort_error) · 출발 거부 · 제한 시간 · 값 거부 · Ctrl+C
  → /mpc/arm false 를 내고 끝. 남은 회차는 --skip K (앞 K 다리 건너뜀) 로 이어서.
기록: /seq/status (String JSON, latched) — 다리마다 {i, kind data|return, ctrl, round, dir, event}. bag 에 남아 tools/bag_runs.py 가 회차에 붙인다.
"""
import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

ORDER = ("mpc", "pp", "adpp")
ROTATE = {"shape": "rotate", "seg_angle_deg": 180.0, "seg_out_and_back": False}     # 복귀 다리 (zigzag)


def plan(shape, n, order=ORDER, return_ctrl="mpc"):
    """다리 목록. 회차 r 마다 order 순서로 data, zigzag 면 data 뒤마다 return (180° 회전).
    dir: zigzag data 는 fwd (출발점 A → B) · rev (B → A) 번갈아, 그 밖 None."""
    legs, k = [], 0
    zz = str(shape).strip().lower() == "zigzag"
    for r in range(1, n + 1):
        for c in order:
            k += 1
            d = ("fwd" if k % 2 == 1 else "rev") if zz else None
            legs.append({"kind": "data", "ctrl": c, "round": r, "dir": d, "data_i": k})
            if zz:
                legs.append({"kind": "return", "ctrl": return_ctrl, "round": r, "dir": d, "data_i": k})
    for i, leg in enumerate(legs, 1):
        leg["i"] = i
    return legs


def compose(a, d):
    """포즈 a (x, y, yaw) 에서 몸체 좌표 변위 d (dx, dy, dyaw) 만큼 간 포즈."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], a[2] + d[2])


def leg_origin(leg, A, end_offset):
    """다리의 출발점 (지도 x, y, yaw). end_offset = 경로 끝의 출발점 기준 변위 (zigzag 만 쓴다)."""
    if leg["dir"] is None:
        return A
    B = compose(A, end_offset)
    flip = (0.0, 0.0, math.pi)
    if leg["kind"] == "data":
        return A if leg["dir"] == "fwd" else compose(B, flip)
    return (B[0], B[1], A[2]) if leg["dir"] == "fwd" else (A[0], A[1], A[2] + math.pi)   # 끝 자리·끝 방향에서 180° 돈다


def origin_params(o):
    yaw = math.degrees(math.atan2(math.sin(o[2]), math.cos(o[2])))
    return {"origin_mode": "fixed", "origin_x": float(o[0]), "origin_y": float(o[1]), "origin_yaw_deg": float(yaw)}


def end_offset_of(cfg, v_limit, budget, k_geom):
    """기준 값 cfg 의 경로 끝 포즈 (출발점 (0, 0, 0) 기준)."""
    from amr_mpc.core import ref_config
    path, _, _ = ref_config.build(cfg, v_limit, budget, k_geom)
    X, _ = path.at(path.T_total + 1.0)
    return float(X[0]), float(X[1]), float(X[2])


def default_n(shape):
    """제어기마다 회차 수. 2026-11-28 사용자 결정 "기본값 6 으로" → 모양과 무관하게 6 (zigzag fwd·rev 3:3, 끝나면 A).
    (그 전: 세 경로 5 → 6 → zigzag 6 · 그 밖 5.) 5 바퀴는 --n 5."""
    return 6


class Abort(Exception):
    pass


def run(a):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter, parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool, String

    from amr_mpc.core import ref_config
    from amr_mpc.body import mpc as mpc_mod

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    REF, SEL, PP, FAULT = "/figure8_reference_node", "/controller_select_node", "/pure_pursuit_node", "/fault_monitor_node"

    class Seq(Node):
        def __init__(self):
            super().__init__(f"seq_{os.getpid()}")
            self.cl = {n: AsyncParameterClient(self, n) for n in (REF, SEL, PP, FAULT)}
            self.status, self.active, self.armed, self.odom = None, None, None, None
            self.faults = []                          # (수신 시각, 이유)
            self.t_start = time.time()
            self.create_subscription(String, "/figure8/status", lambda m: setattr(self, "status", json.loads(m.data)), latched)
            self.create_subscription(String, "/ctrl/active", lambda m: setattr(self, "active", json.loads(m.data)), latched)
            self.create_subscription(Bool, "/mpc/armed", lambda m: setattr(self, "armed", bool(m.data)), latched)
            self.create_subscription(String, "/fault_monitor/fault", lambda m: self.faults.append((time.time(), m.data)), latched)
            self.create_subscription(Odometry, "/state_estimator/odom", self.on_odom, 10)
            self.pub_arm = self.create_publisher(Bool, "/mpc/arm", 10)
            self.pub_seq = self.create_publisher(String, "/seq/status", latched)

        def on_odom(self, m):
            p, q, tw = m.pose.pose, m.pose.pose.orientation, m.twist.twist
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.odom = (time.time(), (p.position.x, p.position.y, yaw), math.hypot(tw.linear.x, tw.linear.y), abs(tw.angular.z))

        def spin_for(self, s):
            t_end = time.time() + s
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, t_end - time.time())))

        def wait(self, cond, timeout, what):
            t_end = time.time() + timeout
            while time.time() < t_end:
                rclpy.spin_once(self, timeout_sec=0.05)
                r = cond()
                if r:
                    return r
            raise Abort(f"{what} — {timeout:.0f} s 안에 안 됨")

        def fut(self, f, timeout=8.0):
            rclpy.spin_until_future_complete(self, f, timeout_sec=timeout)
            if not f.done():
                raise Abort("parameter 서비스 응답 없음")
            return f.result()

        def get(self, node, names):
            if not self.cl[node].wait_for_services(timeout_sec=10.0):
                raise Abort(f"{node} 가 없다 — amrall3 (controller:=all) 로 띄웠나?")
            res = self.fut(self.cl[node].get_parameters(list(names)))
            return {n: (v.type, parameter_value_to_python(v)) for n, v in zip(names, res.values)}

        def set(self, node, values):
            cur = self.get(node, list(values))
            ps = []
            for k, v in values.items():
                typ = cur[k][0]
                if typ == 0:
                    raise Abort(f"{node} 에 {k} 가 없다")
                T = Parameter.Type(typ)
                v = float(v) if T == Parameter.Type.DOUBLE else v
                ps.append(Parameter(k, T, v))
            r = self.fut(self.cl[node].set_parameters_atomically(ps)).result
            if not r.successful:
                raise Abort(f"{node} 거부: {r.reason}")

        def say(self, event, leg=None, **kw):
            st = {"t": time.time(), "event": event, **({k: leg[k] for k in ("i", "kind", "ctrl", "round", "dir", "data_i")} if leg else {}), **kw}
            self.pub_seq.publish(String(data=json.dumps(st, ensure_ascii=False)))

        def disarm(self):
            for _ in range(3):
                self.pub_arm.publish(Bool(data=False))
                self.spin_for(0.05)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C = KeyboardInterrupt, context 는 살아 있어 /mpc/arm false 를 낼 수 있다
    s = Seq()
    try:
        s.spin_for(1.0)                                  # latched 받기
        old_faults = len(s.faults)
        live = s.get(REF, list(ref_config.LIVE))
        cfg = {k: (list(v) if isinstance(v, (list, tuple)) else v) for k, (_, v) in live.items()}
        shape = str(cfg["shape"]).strip().lower()
        n = a.n or default_n(shape)
        order = [c.strip().lower() for c in a.order.split(",") if c.strip()]
        legs = plan(shape, n, order, a.return_ctrl)
        lim = s.get(REF, ["load_budget", "max_wheel_rad_per_sec"])
        budget = lim["load_budget"][1]
        v_limit = mpc_mod.R_WHEEL * float(lim["max_wheel_rad_per_sec"][1])     # 기준 노드와 같게 (set_wheel_limit)
        end = end_offset_of(cfg, v_limit, budget, mpc_mod.K_GEOM) if shape == "zigzag" else (0.0, 0.0, 0.0)
        if shape == "zigzag" and int(cfg["zigzag_n"]) % 2 == 0 and not a.allow_even:
            # 2026-11-28: 직선 수가 짝수면 180° 뒤 rev 회차가 다른 바닥을 지난다 (n 4 · L 2.5 계산: 두 경로 최대 2.18 m 떨어짐). 홀수면 같은 선 (0.00 cm)
            raise Abort(f"zigzag_n={cfg['zigzag_n']} (짝수) — 180° 복귀 뒤 rev 회차가 fwd 와 다른 바닥을 지난다 (미리 본 공간 밖). "
                        "홀수로 (pit s2 zigzag_n=5) 하거나, 알고 쓰려면 --allow-even")
        la = s.get(PP, ["lookahead_time", "lookahead_min", "lookahead_max",
                        "adpp_lookahead_time", "adpp_lookahead_min", "adpp_lookahead_max"])
        mte = s.get(FAULT, ["max_track_error"])["max_track_error"][1]
        st = s.status or {}
        print(f"기준: {st.get('next_summary', '?')}")
        print(f"PP 룩어헤드 pp [{la['lookahead_time'][1]:g} s, {la['lookahead_min'][1]:g}~{la['lookahead_max'][1]:g} m] · "
              f"adpp [{la['adpp_lookahead_time'][1]:g} s, {la['adpp_lookahead_min'][1]:g}~{la['adpp_lookahead_max'][1]:g} m] · "
              f"fault 추종 오차 한계 {mte*100:.0f} cm")
        nd = sum(1 for g in legs if g["kind"] == "data")
        print(f"순서: {' → '.join(order)} × {n} = data {nd} 회차" + (f" + 복귀 180° 회전 {len(legs) - nd} 번 ({a.return_ctrl})" if shape == "zigzag" else "")
              + f", anchor {'켬' if a.anchor else '끔'}" + (f", 앞 {a.skip} 다리 건너뜀" if a.skip else ""))
        if shape == "zigzag":
            print(f"  zigzag 끝 변위 ({end[0]:+.3f}, {end[1]:+.3f}) m, {math.degrees(end[2]):+.1f}°")
        if a.dry:
            for g in legs:
                print(f"  {g['i']:3d}  {g['kind']:6s} {g['ctrl']:4s} 회차 {g['round']}" + (f" {g['dir']}" if g["dir"] else ""))
            return 0
        if s.armed is None or s.armed:
            raise Abort("게이트가 무장 중이거나 /mpc/armed 를 못 받음 — 's' 로 정지한 뒤")
        if s.odom is None or time.time() - s.odom[0] > 0.5:
            s.wait(lambda: s.odom and time.time() - s.odom[0] < 0.5, 3.0, "/state_estimator/odom")
        if a.anchor_pose:
            x, y, yd = (float(v) for v in a.anchor_pose.split(","))
            A = (x, y, math.radians(yd))
            src = "--anchor-pose"
        else:
            A, src = s.odom[1], "지금 로봇 자리"
        print(f"출발점 A = ({A[0]:+.3f}, {A[1]:+.3f}) m, {math.degrees(A[2]):+.1f}° ({src}) — "
              f"이어서 돌 때: --anchor-pose {A[0]:.3f},{A[1]:.3f},{math.degrees(A[2]):.2f}")
        s.anchor_A = A
        for k in range(int(a.countdown), 0, -1):
            print(f"  {k} s 뒤 시작 — 멈추려면 Ctrl+C", end="\r", flush=True)
            s.spin_for(1.0)
        print()
        s.say("start", n=n, order=order, shape=shape, anchor=a.anchor, A=A, legs=len(legs))
        base = {k: cfg[k] for k in ("shape", *ROTATE, "origin_mode", "origin_x", "origin_y", "origin_yaw_deg")}

        for leg in legs:
            if leg["i"] <= a.skip:
                continue
            tag = f"[{leg['i']}/{len(legs)}] {leg['kind']} {leg['ctrl']} 회차 {leg['round']}" + (f" {leg['dir']}" if leg["dir"] else "")
            s.wait(lambda: s.armed is False, 5.0, "정지 확인")
            # 1. 제어기 · 기준 값
            if not s.active or s.active.get("active") != leg["ctrl"]:
                s.set(SEL, {"active": leg["ctrl"]})
                s.wait(lambda: s.active and s.active.get("active") == leg["ctrl"], 3.0, "제어기 선택 반영")
            vals = dict(ROTATE) if leg["kind"] == "return" else {k: base[k] for k in ("shape", *ROTATE)}
            if a.anchor:
                vals.update(origin_params(leg_origin(leg, A, end)))
            else:
                vals.update({k: base[k] for k in ("origin_mode", "origin_x", "origin_y", "origin_yaw_deg")})
            s.set(REF, vals)
            run_before = s.wait(lambda: s.status and s.status.get("state") == "idle" and s.status, 5.0, "기준 idle")["run"]
            # 2. 정지 확인
            t_still = None
            t_end = time.time() + 20.0
            while True:
                rclpy.spin_once(s, timeout_sec=0.05)
                o = s.odom
                still = o is not None and time.time() - o[0] < 0.3 and o[2] < 0.03 and o[3] < 0.05
                t_still = (t_still or time.time()) if still else None
                if t_still and time.time() - t_still >= a.settle:
                    break
                if time.time() > t_end:
                    raise Abort("로봇이 20 s 안에 서지 않음")
            # 3. 무장
            print(f"{tag} — 무장")
            s.say("arm", leg)
            s.pub_arm.publish(Bool(data=True))
            s.wait(lambda: s.armed, 3.0, "/mpc/armed true")
            stv = s.wait(lambda: s.status and s.status.get("run") == run_before + 1 and s.status.get("state") == "running" and s.status,
                         6.0, "기준 출발 (출발점에서 30 cm · 30° 밖이면 기준 노드가 거부)")
            limit = float(stv.get("T_total", 60.0)) + max(0.0, float(stv.get("hold_s", 5.0))) + 30.0
            # 4. 완주 기다림
            t_end = time.time() + limit
            while True:
                rclpy.spin_once(s, timeout_sec=0.05)
                sv = s.status or {}
                if sv.get("run") == run_before + 1 and sv.get("state") == "finished":
                    if sv.get("end") != "done":
                        raise Abort(f"기준 중단 ({sv.get('end') or '까닭 모름'}) — abort_error 이탈")
                    break
                if not s.armed:
                    new = s.faults[old_faults:]
                    raise Abort(f"회차 도중 무장 꺼짐 — {new[-1][1] if new else 'fault 기록 없음 (s · 패드 B 로 사람이 멈춤?)'}")
                if time.time() > t_end:
                    raise Abort(f"{limit:.0f} s 안에 안 끝남")
            s.disarm()
            s.wait(lambda: s.armed is False, 3.0, "/mpc/armed false")
            s.say("done", leg, run=run_before + 1)
            print(f"{tag} — 끝 (기준 판 #{run_before + 1})")
        s.set(REF, {k: base[k] for k in ("shape", *ROTATE)})        # 마지막에 원래 경로 값으로
        s.say("end")
        print("순서 끝 — 모두 완주")
        return 0
    except (Abort, KeyboardInterrupt) as e:
        s.disarm()
        why = "Ctrl+C" if isinstance(e, KeyboardInterrupt) else str(e)
        try:
            s.say("abort", reason=why)
            s.spin_for(0.2)
        except Exception:
            pass
        A = getattr(s, "anchor_A", None)
        print(f"\n✗ 순서 중단: {why}\n  /mpc/arm false 를 냈다. 다시 무장하지 않는다.\n"
              f"  이어서: 로봇을 그 다리의 출발점에 놓고 amrseq --skip <끝난 다리 수>"
              + (f" --anchor-pose {A[0]:.3f},{A[1]:.3f},{math.degrees(A[2]):.2f}" if A else "")
              + " (경로가 바뀌었으면 rqt·pit 로 되돌린 뒤)")
        return 1
    finally:
        s.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0], formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--n", type=int, default=0, help="제어기마다 회차 수 (0 = 기본 6)")
    ap.add_argument("--order", default=",".join(ORDER), help="제어기 순서 (기본 mpc,pp,adpp)")
    ap.add_argument("--return-ctrl", default="mpc", help="zigzag 복귀 회전을 할 제어기 (기본 mpc)")
    ap.add_argument("--settle", type=float, default=2.0, help="무장 전 로봇이 서 있어야 하는 시간 [s]")
    ap.add_argument("--countdown", type=float, default=5.0, help="시작 전 대기 [s]")
    ap.add_argument("--skip", type=int, default=0, help="앞 K 다리 건너뜀 (중단 뒤 이어서)")
    ap.add_argument("--no-anchor", dest="anchor", action="store_false", help="출발점 고정 끔 (무장한 자리 = 출발점)")
    ap.add_argument("--anchor-pose", default="", help="출발점 A 를 x,y,yaw_deg 로 (중단 뒤 --skip 과 같이 — 처음 출력된 값)")
    ap.add_argument("--dry", action="store_true", help="할 일만 출력")
    ap.add_argument("--allow-even", action="store_true", help="zigzag_n 짝수도 (rev 회차가 다른 바닥을 지난다 — 공간 확인 뒤)")
    a = ap.parse_args(argv)
    for c in a.order.split(","):
        if c.strip() and c.strip().lower() not in ORDER:
            ap.error(f"--order {c} — {', '.join(ORDER)}")
    return run(a)


if __name__ == "__main__":
    sys.exit(main())
