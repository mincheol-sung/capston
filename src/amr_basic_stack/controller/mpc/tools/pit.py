#!/usr/bin/env python3
"""pit — 실차 토크 실험 스택을 켠 채로 다음 판 조건을 바꾼다 (2026-11-28 pitwall, amr/notes_amr/pitwall_design.md).

  pit                       지금 상태 (= pit show)
  pit show                  판 번호·상태, 다음 판 기준 요약·받아 둔 변경·마지막 거부, MPC 값
  pit presets               프리셋 목록 (config/pitwall_presets.yaml)
  pit <프리셋> [k=v ...]     프리셋 적용 (예: pit circ, pit poly accel=1.2)
  pit set k=v [k=v ...]     값 바꾸기 — 기준·MPC 값을 이름으로 나눠 노드마다 한 번에 (set_parameters_atomically)
  pit new                   멈춘 판을 버리고 다음 'a' 를 새 판으로 (정지 중만)
  pit dump                  지금 다음 판 기준·MPC 값을 프리셋 yaml 꼴로 출력

적용 시점: 기준 값은 다음 판부터 (진행 중인 판은 그대로), MPC 값은 정지 중이면 바로, 무장 중이면 다음 정지 때.
controller:=all (2026-11-28) 에서는 PP 룩어헤드 (lookahead_*, adpp_lookahead_*) · fault 추종 오차 한계 (max_track_error) ·
  제어기 (active) 도 이름으로 나눠 보낸다 (정지 중만). 프리셋 s1 · s2 · s3 = 11-26 세 경로 조건 (config/pitwall_presets.yaml).
검사에 걸리면 (부하 예산 등) 노드가 거부하고 이유를 돌려준다 — 아무 값도 안 바뀐다.
이 도구는 무장하지 않는다 ('a' · RT 는 사람이 직접).
"""
import json
import os
import sys
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config")
sys.path.insert(0, os.path.join(HERE, ".."))
from amr_mpc.core.ref_config import LIVE as REF_LIVE  # noqa: E402
from amr_mpc.core import presets as presets_mod  # noqa: E402

REF_NODE, MPC_NODE = "/figure8_reference_node", "/torque_mpc_node"
PP_NODE, FAULT_NODE, SEL_NODE = "/pure_pursuit_node", "/fault_monitor_node", "/controller_select_node"   # 2026-11-28 controller:=all
PP_LIVE = ("lookahead_time", "lookahead_min", "lookahead_max", "adpp_lookahead_time", "adpp_lookahead_min", "adpp_lookahead_max")
FAULT_LIVE, SEL_LIVE = ("max_track_error",), ("active",)
MPC_LIVE = ("W", "w_dudt", "r_u", "q_null", "mu_hat", "slip_fb_enable", "slip_fb_scale", "slip_eta_enable",
            "disturbance_enable", "disturbance_alpha", "roller_rr", "inertia_z")   # torque_mpc_node.cpp kLiveParams
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def load_presets():
    return presets_mod.load_presets(CONFIG)


def parse_kv(items):
    out = {}
    for it in items:
        k, sep, v = it.partition("=")
        if not sep or not k:
            raise SystemExit(f"pit: '{it}' — key=value 꼴로 (예: accel=1.2, W=[80,80,10,2,2,1])")
        out[k.strip()] = yaml.safe_load(v) if v.strip() else ""
    return out


class Pit(Node):
    def __init__(self):
        super().__init__(f"pit_{os.getpid()}")
        self.cl = {n: AsyncParameterClient(self, n) for n in (REF_NODE, MPC_NODE, PP_NODE, FAULT_NODE, SEL_NODE)}
        self.latest = {}
        for topic in ("/figure8/status", "/torque_mpc/config"):
            self.create_subscription(String, topic, lambda m, t=topic: self.latest.__setitem__(t, json.loads(m.data)), LATCHED)

    def wait(self, fut, timeout=8.0):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done():
            raise SystemExit("pit: 응답 없음 — 스택이 떠 있나? (같은 ROS_DOMAIN_ID, 다른 컴퓨터면 AMR_PEERS)")
        return fut.result()

    def ready(self, node, timeout=10.0):        # 2026-11-28 5 → 10 s: 실차 poly 스택에서 fault_monitor 를 5 s 안에 못 찾은 일 (원인 미확인)
        return self.cl[node].wait_for_services(timeout_sec=timeout)

    def get(self, node, names):
        res = self.wait(self.cl[node].get_parameters(list(names)))
        return {n: (v.type, parameter_value_to_python(v)) for n, v in zip(names, res.values)}

    def set_atomic(self, node, values):
        """노드의 지금 타입에 맞춰 바꾼 뒤 한 번에 set. (성공, 이유)."""
        cur = self.get(node, list(values))
        params = []
        for k, v in values.items():
            typ = cur[k][0]
            if typ == 0:                                  # NOT_SET = 없는 이름
                return False, f"{k}: {node} 에 없는 parameter"
            params.append(Parameter(k, Parameter.Type(typ), coerce(k, v, typ)))
        res = self.wait(self.cl[node].set_parameters_atomically(params))
        r = res.result
        return r.successful, r.reason

    def status(self, timeout=2.0):
        t_end = time.time() + timeout
        while time.time() < t_end and len(self.latest) < 2:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.latest


def coerce(k, v, typ):
    T = Parameter.Type
    try:
        if typ == T.DOUBLE.value:
            return float(v)
        if typ == T.INTEGER.value:
            if float(v) != int(float(v)):
                raise ValueError
            return int(float(v))
        if typ == T.BOOL.value:
            if isinstance(v, bool):
                return v
            s = str(v).strip().lower()
            if s not in ("true", "false", "1", "0", "on", "off"):
                raise ValueError
            return s in ("true", "1", "on")
        if typ == T.STRING.value:
            return str(v)
        if typ == T.DOUBLE_ARRAY.value:
            return [float(x) for x in (v if isinstance(v, (list, tuple)) else str(v).strip("[] ").split(","))]
    except (TypeError, ValueError):
        raise SystemExit(f"pit: {k}={v!r} — 타입이 맞지 않는다 ({T(typ).name})")
    return v


GROUPS = ((REF_NODE, REF_LIVE, "기준"), (MPC_NODE, MPC_LIVE, "MPC"), (PP_NODE, PP_LIVE, "PP"),
          (FAULT_NODE, FAULT_LIVE, "fault"), (SEL_NODE, SEL_LIVE, "제어기"))


def split(values):
    """이름 → 노드별 묶음 {노드: {이름: 값}}. 겹치는 이름은 없다 (기준 accel · PP lookahead_* · …)."""
    out = {node: {k: v for k, v in values.items() if k in names} for node, names, _ in GROUPS}
    bad = [k for k in values if not any(k in names for _, names, _ in GROUPS)]
    if bad:
        raise SystemExit(f"pit: 실행 중 바꿀 수 없는 이름 {bad}\n"
                         + "".join(f"  {what}: {', '.join(names)}\n" for _, names, what in GROUPS)
                         + "  (그 밖의 값은 launch 인자로 — 다시 띄울 것)")
    return out


def preset_groups(name, p, rest):
    """프리셋 → split() 과 같은 꼴. 기준은 track_base + reference, 나머지는 적은 것만. rest (k=v) 가 이긴다."""
    vals = dict(presets_mod.reference_values(CONFIG, name))
    for key in ("mpc", "pp", "fault"):
        vals.update(p.get(key) or {})
    vals.update(parse_kv(rest))
    return split(vals)


def apply(pit, groups):
    ok_all = True
    for node, _, what in GROUPS:
        vals = groups.get(node)
        if not vals:
            continue
        if not pit.ready(node):
            print(f"✗ {what}: {node} 가 없다 (스택이 떠 있나, controller:=mpc · all 인가)")
            ok_all = False
            continue
        ok, reason = pit.set_atomic(node, vals)
        shown = ", ".join(f"{k}={v}" for k, v in vals.items()) if len(vals) <= 8 else f"{len(vals)} 개 값"
        print(f"{'✓' if ok else '✗ 거부'} {what}: {shown}" + ("" if ok else f"\n    이유: {reason}"))
        ok_all &= ok
    return ok_all


def show(pit):
    st = pit.status()
    s = st.get("/figure8/status")
    if s:
        word = {"idle": "판 없음 (다음 'a' 가 새 판)", "running": f"판 #{s['run']} 주행 중", "paused": f"판 #{s['run']} 멈춤",
                "finished": f"판 #{s['run']} 끝 ('s' 뒤 'a' = 새 판)"}.get(s["state"], s["state"])
        print(f"기준: {word} | 다음 판 #{s['next_run']}")
        print(f"  다음 판: {s['next_summary']}")
        if s["pending"]:
            print("  받아 둔 변경 (지금 판에는 안 들어감): " + ", ".join(f"{k} {a}→{b}" for k, (a, b) in s["pending"].items()))
        if s["last_reject"]:
            print(f"  마지막 거부: {s['last_reject']}")
    else:
        print("기준: /figure8/status 없음 (기준 노드가 안 떠 있거나 pitwall 전 판)")
    c = st.get("/torque_mpc/config")
    if c:
        L, F = c["live"], c["launch_only"]
        print(f"MPC: W {L['W']} w_dudt {L['w_dudt']:g} r_u {L['r_u']:g} q_null {L['q_null']:g} mu_hat {L['mu_hat']:g} "
              f"slip_fb {'켬' if L['slip_fb_enable'] else '끔'} ×{L['slip_fb_scale']:g} slip_eta {'켬' if L['slip_eta_enable'] else '끔'} "
              f"외란 {'켬' if L['disturbance_enable'] else '끔'} α {L['disturbance_alpha']:g} roller {L['roller_rr']:g} I_z {L['inertia_z']:g}")
        print(f"  launch 때 값: limit_mode {F['limit_mode']}, Ts {F['Ts']:g}, Np {F['Np']}, Nc {F['Nc']}, cap_on {F['cap_on']}, "
              f"velocity_source {F['velocity_source']}")
        if c["pending"]:
            print("  받아 둔 값 (다음 정지 때 적용): " + ", ".join(f"{k} {L[k]}→{v}" for k, v in c["pending"].items() if L[k] != v))
        if c["last_reject"]:
            print(f"  마지막 거부: {c['last_reject']}")
    else:
        print("MPC: /torque_mpc/config 없음 (controller:=mpc 가 아니거나 pitwall 전 빌드)")


def check_requires(pit, req):
    if not req:
        return
    st = pit.status()
    c = st.get("/torque_mpc/config")
    if req.get("controller") in ("mpc", "all") and not c:
        print(f"⚠ 이 프리셋은 controller:={req['controller']} 용인데 MPC 노드가 안 보인다 (MPC 값은 못 바꾼다)")
    if req.get("controller") == "all" and not pit.ready(SEL_NODE, timeout=5.0):
        # 2026-11-28: 일부만 바꾸고 실패하던 것 (poly 스택에 pit s1 → 기준·MPC 만 s1) → 아무것도 안 바꾸고 끝낸다
        raise SystemExit("✗ 이 프리셋은 controller:=all 스택용인데 controller_select_node 가 없다 — 아무 값도 안 바꿨다.\n"
                         "  amrall3 bag:=<이름> 으로 띄운 뒤 다시 (지금 스택: amrall · poly · circ 같은 controller:=mpc 판)")
    if c:
        for k, v in req.items():
            if k in c["launch_only"] and str(c["launch_only"][k]) != str(v):
                print(f"⚠ 이 프리셋은 {k}={v} 판인데 지금 {c['launch_only'][k]} — launch 때만 바뀐다 (다시 띄울 것)")


def main(argv):
    if argv and argv[0] in ("-h", "--help", "help"):
        print(__doc__)
        return 0
    cmd, rest = (argv[0], argv[1:]) if argv else ("show", [])
    presets = load_presets()
    if cmd == "presets":
        for name, p in presets.items():
            print(f"{name:8s} {p.get('description', '')}\n         기준 {p.get('reference', {})}  MPC {p.get('mpc', {})}"
              + (f"  PP {p['pp']}" if p.get("pp") else "") + (f"  fault {p['fault']}" if p.get("fault") else ""))
        return 0
    rclpy.init()
    pit = Pit()
    try:
        if cmd == "show":
            show(pit)
        elif cmd == "set":
            if not rest:
                raise SystemExit("pit set k=v [k=v ...]")
            ok = apply(pit, split(parse_kv(rest)))
            print()
            show(pit)
            return 0 if ok else 1
        elif cmd == "new":
            cli = pit.create_client(Trigger, f"{REF_NODE}/new_run")
            if not cli.wait_for_service(timeout_sec=5.0):
                raise SystemExit("pit: new_run 서비스 없음 (기준 노드가 pitwall 전 판이거나 안 떠 있다)")
            r = pit.wait(cli.call_async(Trigger.Request()))
            print(("✓ " if r.success else "✗ ") + r.message)
            return 0 if r.success else 1
        elif cmd == "dump":
            st = pit.status()
            s, c = st.get("/figure8/status"), st.get("/torque_mpc/config")
            d = {"my_preset": {"description": "pit dump " + time.strftime("%Y-%m-%d %H:%M"),
                               "reference": s["next"] if s else {}, "mpc": c["live"] if c else {}}}
            if c:
                d["my_preset"]["requires"] = {"limit_mode": c["launch_only"]["limit_mode"]}
            print(yaml.safe_dump(d, allow_unicode=True, sort_keys=False, default_flow_style=None))
        elif cmd in presets:
            p = presets[cmd]
            check_requires(pit, p.get("requires"))
            print(f"프리셋 {cmd}: {p.get('description', '')}")
            ok = apply(pit, preset_groups(cmd, p, rest))
            print()
            show(pit)
            return 0 if ok else 1
        else:
            raise SystemExit(f"pit: '{cmd}' — show | presets | set | new | dump | 프리셋 이름 ({', '.join(presets)}). pit -h")
    finally:
        pit.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
