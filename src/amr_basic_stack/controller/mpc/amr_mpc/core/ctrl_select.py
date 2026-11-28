# controller:=all 제어기 선택 — 순수 함수 (2026-11-28, ROS 없이 시험: test/test_ctrl_select.py)
# ---------------------------------------------------------------
# controller_select_node 가 /mpc/armed 를 고른 제어기 묶음에만 넘기고 /ctrl/active (String JSON, latched) 를 낸다.
# torque_output · fault_monitor · pure_pursuit 은 /ctrl/active 를 보고 고른 제어기의 토픽·값만 쓴다 (nodes/controller_select.py 머리주석).
# ---------------------------------------------------------------
import json

CONTROLLERS = ("mpc", "pp", "adpp")


def family(active):
    """제어기 이름 → 무장 신호를 받는 노드 묶음. pp·adpp 는 같은 pure_pursuit_node + wheel_pi_node."""
    a = str(active).strip().lower()
    if a not in CONTROLLERS:
        raise ValueError(f"active={active!r} — {' | '.join(CONTROLLERS)}")
    return "mpc" if a == "mpc" else "pp"


def route(active, armed):
    """(고른 제어기, 게이트 무장) → {묶음: 그 묶음에 보낼 무장 값}. 고르지 않은 묶음은 늘 False."""
    f = family(active)
    return {"mpc": bool(armed) and f == "mpc", "pp": bool(armed) and f == "pp"}


def parse_active(data):
    """/ctrl/active 의 String (JSON) → 제어기 이름 (틀리면 ValueError)."""
    try:
        a = str(json.loads(data)["active"]).strip().lower()
    except (TypeError, KeyError, json.JSONDecodeError) as e:
        raise ValueError(f"/ctrl/active 를 못 읽음: {data!r} ({e})")
    family(a)
    return a


def parse_map(items):
    """["mpc=/torque_mpc/cmd_raw", "pp=/wheel_pi/cmd_raw", ...] → {제어기: 토픽}. 세 제어기가 다 있어야 한다."""
    out = {}
    for it in items:
        k, sep, v = str(it).partition("=")
        k, v = k.strip().lower(), v.strip()
        if not sep or not v:
            raise ValueError(f"'{it}' — 제어기=토픽 꼴로 (예: pp=/wheel_pi/cmd_raw)")
        family(k)
        out[k] = v
    missing = [c for c in CONTROLLERS if c not in out]
    if missing:
        raise ValueError(f"제어기 {missing} 의 토픽이 없다 — {list(items)}")
    return out


class TopicSelect:
    """같은 꼴 토픽 여럿 중 고른 제어기의 것만 통과. 고르기 전 (/ctrl/active 를 아직 못 받음) 에는 아무것도 안 통과."""

    def __init__(self, items):
        self.map = parse_map(items)
        self.active = None

    def topics(self):
        return sorted(set(self.map.values()))

    def select(self, active):
        family(active)
        self.active = str(active).strip().lower()
        return self.map[self.active]

    def passes(self, topic):
        return self.active is not None and self.map[self.active] == topic
