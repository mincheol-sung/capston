#!/usr/bin/env python3
"""bag 하나에 든 여러 판을 나눈다 (2026-11-28 pitwall — launch 하나 = bag 하나, 판마다 조건이 bag 에 남는다).

  python3 tools/bag_runs.py BAG [BAG ...]          판 표: 시작·끝·끝난 이유·기준 요약·MPC 값·앞 판과 다른 값
  python3 tools/bag_runs.py BAG --json runs.json   같은 내용을 JSON 으로 (분석 스크립트용)
  python3 tools/bag_runs.py BAG --changes          판 사이에 받은·거부한 변경 기록도

판을 나누는 근거 (bag 의 토픽, 모두 latched JSON)
  /figure8/run_config   새 판이 출발할 때 그 판의 기준 값 전체·판 번호·출발 포즈 (figure8_reference_node)
  /figure8/status       상태가 바뀔 때마다 (finished = 완주·중단, idle = 판 지움) · 변경 받음·거부
  /torque_mpc/config    MPC 값: 기동·변경 적용·거부·무장 때마다 (event arm = 그 판에서 쓴 값)
  /mpc/armed            무장 구간 (판 안에서 멈췄다 이어 가면 무장 구간이 여럿)
  /ctrl/active          controller:=all 에서 고른 제어기 (2026-11-28, controller_select_node) → r["controller"]
  /seq/status           amrseq 다리 (2026-11-28, tools/seq.py) → r["seq"] = {i, kind data|return, ctrl, round, dir, data_i}
  controller:=all 이 아닌 bag 은 r["controller"] = None (그 launch 의 controller 인자 하나 — run_info/args.txt)
판 n 의 창 = [run_config n 수신 시각, 판 n 이 finished 가 된 시각 | 판 n 이 버려진 시각 | 다음 판 출발 | bag 끝].

다른 스크립트에서:
    from bag_runs import runs          # sys.path 에 tools/ 를 넣고
    for r in runs(bag):                # r["t0"], r["t1"] = bag 수신 시각 [s] (rosbag 타임스탬프, /mpc/armed 수신 시각과 같은 축)
        ...                            # r["ref"] = run_config dict, r["mpc"] = MPC live 값, r["armed"] = [(t_on, t_off), …]
pitwall 전 bag (run_config 없음) 은 무장 구간 전체를 판 1 개로 돌려준다 (예전 분석과 같음).
"""
import argparse
import json
import sys

TOPICS = ["/figure8/run_config", "/figure8/status", "/torque_mpc/config", "/mpc/armed", "/ctrl/active", "/seq/status"]


def read(bag):
    """[(수신 시각 s, 토픽, 값)] — JSON 토픽은 dict, /mpc/armed 는 bool."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    out = []
    while r.has_next():
        tp, data, tb = r.read_next()
        m = deserialize_message(data, get_message(types[tp]))
        out.append((tb * 1e-9, tp, bool(m.data) if tp == "/mpc/armed" else json.loads(m.data)))
    meta = rosbag2_py.Info().read_metadata(bag, "")
    t_start = meta.starting_time.nanoseconds * 1e-9
    t_end = t_start + meta.duration.nanoseconds * 1e-9
    return sorted(out, key=lambda e: e[0]), t_start, t_end


def armed_spans(events, t0, t1):
    """/mpc/armed 사건 → [t0, t1] 안의 무장 구간 [(on, off)]."""
    spans, on = [], None
    for t, tp, v in events:
        if tp != "/mpc/armed":
            continue
        if v and on is None:
            on = t
        elif not v and on is not None:
            spans.append((on, t))
            on = None
    if on is not None:
        spans.append((on, float("inf")))
    return [(max(a, t0), min(b, t1)) for a, b in spans if b > t0 and a < t1]


def runs(bag):
    """판 목록 (모듈 머리주석). 시각은 bag 수신 시각 [s]."""
    return split_runs(*read(bag))


def split_runs(ev, t_start, t_end):
    """read() 의 사건 목록 → 판 목록 (bag 없이 시험 — test/test_bag_runs.py)."""
    starts = [(t, v) for t, tp, v in ev if tp == "/figure8/run_config"]
    mpc_msgs = [(t, v) for t, tp, v in ev if tp == "/torque_mpc/config"]
    if not starts:                                    # pitwall 전 bag
        sp = armed_spans(ev, t_start, t_end)
        if not sp:
            return []
        return [{"run": 1, "t0": sp[0][0], "t1": min(sp[-1][1], t_end), "end": "bag 끝 (run_config 없음 — pitwall 전 bag)",
                 "ref": None, "mpc": mpc_msgs[-1][1]["live"] if mpc_msgs else None, "armed": sp}]
    out = []
    for i, (t0, rc) in enumerate(starts):
        nxt = starts[i + 1][0] if i + 1 < len(starts) else t_end
        t1, end = nxt, "다음 판 출발" if i + 1 < len(starts) else "bag 끝"
        for t, tp, v in ev:                           # 이 판이 끝났거나 버려진 시각
            if tp != "/figure8/status" or t <= t0 or t >= nxt:
                continue
            if v.get("run") == rc["run"] and v.get("state") == "finished":
                t1, end = t, "완주·중단 (finished)"
                break
            if v.get("run") == rc["run"] and v.get("state") == "idle":
                t1, end = t, "버림 (멈춘 판에서 값 변경·new_run)"
                break
        # 이 판의 MPC 값: 출발 직전·직후 (±1 s) 의 arm 사건, 없으면 출발 전 마지막 config
        near = [(t, v) for t, v in mpc_msgs if t <= t0 + 1.0]
        arm = [(t, v) for t, v in near if v.get("event") == "arm" and t >= t0 - 1.0]
        pick = (arm or near or [(None, None)])[-1][1]
        # 2026-11-28 controller:=all: 출발 (run_config) 직전·직후의 /ctrl/active, amrseq 의 arm 사건
        act = [v for t, tp, v in ev if tp == "/ctrl/active" and t <= t0 + 1.0]
        seq = [v for t, tp, v in ev if tp == "/seq/status" and v.get("event") == "arm" and t0 - 10.0 <= t <= t0 + 1.0]
        out.append({"run": rc["run"], "t0": t0, "t1": t1, "end": end, "ref": rc,
                    "mpc": pick["live"] if pick else None, "mpc_launch": pick["launch_only"] if pick else None,
                    "armed": armed_spans(ev, t0, t1),
                    "controller": act[-1]["active"] if act else None,
                    "seq": {k: seq[-1].get(k) for k in ("i", "kind", "ctrl", "round", "dir", "data_i")} if seq else None})
    return out


def changes(bag):
    """판 사이 변경 기록: (시각, 누가, 무엇). 받음 = 다음 판 값 변화, 거부 = last_reject."""
    ev, t_start, _ = read(bag)
    out, prev_next, prev_rej, prev_mrej = [], None, "", ""
    for t, tp, v in ev:
        if tp == "/figure8/status":
            if prev_next is not None and v["next"] != prev_next:
                diff = {k: [prev_next.get(k), x] for k, x in v["next"].items() if prev_next.get(k) != x}
                out.append((t - t_start, "기준 받음", diff))
            if v["last_reject"] and v["last_reject"] != prev_rej:
                out.append((t - t_start, "기준 거부", v["last_reject"]))
            prev_next, prev_rej = v["next"], v["last_reject"]
        elif tp == "/torque_mpc/config":
            if v["event"] in ("applied", "pending"):
                out.append((t - t_start, f"MPC {'적용' if v['event'] == 'applied' else '받아 둠'}",
                            v["live"] if v["event"] == "applied" else v["pending"]))
            elif v["event"] == "reject" and v["last_reject"] != prev_mrej:
                out.append((t - t_start, "MPC 거부", v["last_reject"]))
            prev_mrej = v["last_reject"]
    return out


def _diff(a, b):
    if a is None or b is None:
        return {}
    return {k: [a.get(k), v] for k, v in b.items() if a.get(k) != v}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--json", default="", help="판 목록을 JSON 으로 (bag 이 여럿이면 {bag: [...]})")
    ap.add_argument("--changes", action="store_true", help="판 사이 변경 기록도")
    a = ap.parse_args()
    all_out = {}
    for bag in a.bags:
        bag = bag.rstrip("/")
        rs = runs(bag)
        all_out[bag] = rs
        _, t_start, _ = read(bag) if rs else (None, 0.0, None)
        print(f"\n== {bag}: 판 {len(rs)} 개")
        prev_ref, prev_mpc = None, None
        for r in rs:
            arm_s = sum(min(b, r["t1"]) - a_ for a_, b in r["armed"])
            ref = r["ref"]
            print(f"판 #{r['run']}  {r['t0'] - t_start:7.1f} ~ {r['t1'] - t_start:7.1f} s  (무장 {arm_s:.1f} s, "
                  f"무장 구간 {len(r['armed'])} 개)  끝: {r['end']}")
            if r.get("controller") or r.get("seq"):
                q = r.get("seq") or {}
                print(f"   제어기 {r.get('controller') or '?'}" + (f"   amrseq 다리 {q['i']} {q['kind']} 회차 {q['round']}"
                      + (f" {q['dir']}" if q.get("dir") else "") if q else "")
                      + ("   ⚠ amrseq 제어기와 다름" if q and q.get("ctrl") != r.get("controller") else ""))
            if ref:
                print(f"   기준  {ref['summary']}")
                print(f"         출발 포즈 ({ref['origin'][0]:+.2f}, {ref['origin'][1]:+.2f}, {ref['origin'][2]:+.2f} rad), "
                      f"origin {ref['origin_mode']}")
                d = _diff(prev_ref and prev_ref["relevant"], ref["relevant"]) if prev_ref and prev_ref["shape"] == ref["shape"] else {}
                if prev_ref and prev_ref["shape"] != ref["shape"]:
                    print(f"         앞 판과 모양이 다름 ({prev_ref['shape']} → {ref['shape']})")
                elif d:
                    print("         앞 판과 다른 값: " + ", ".join(f"{k} {x}→{y}" for k, (x, y) in d.items()))
            if r["mpc"]:
                L = r["mpc"]
                print(f"   MPC   W {L['W']} w_dudt {L['w_dudt']:g} r_u {L['r_u']:g} mu_hat {L['mu_hat']:g} "
                      f"slip_fb {'on' if L['slip_fb_enable'] else 'off'}×{L['slip_fb_scale']:g} "
                      f"slip_eta {'on' if L['slip_eta_enable'] else 'off'} 외란 {'on' if L['disturbance_enable'] else 'off'} "
                      f"α {L['disturbance_alpha']:g} roller {L['roller_rr']:g} I_z {L['inertia_z']:g}"
                      + (f"  ({r['mpc_launch']['limit_mode']})" if r.get("mpc_launch") else ""))
                d = _diff(prev_mpc, L)
                if d:
                    print("         앞 판과 다른 MPC 값: " + ", ".join(f"{k} {x}→{y}" for k, (x, y) in d.items()))
            prev_ref, prev_mpc = ref or prev_ref, r["mpc"] or prev_mpc
        if a.changes:
            print("  변경 기록 (bag 시작 기준 s):")
            for t, who, what in changes(bag):
                print(f"   {t:7.1f}  {who}: {what}")
    if a.json:
        data = all_out if len(a.bags) > 1 else all_out[a.bags[0].rstrip("/")]
        with open(a.json, "w") as f:
            json.dump(data, f, ensure_ascii=False, indent=1, default=str)
        print(f"\nJSON: {a.json}")


if __name__ == "__main__":
    sys.exit(main())
