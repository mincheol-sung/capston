# pitwall 프리셋 (2026-11-28) — config/pitwall_presets.yaml. tools/pit.py 와 figure8_reference_node (rqt 의 preset 칸) 가 같이 쓴다.
#   기준 값 = torque_track.yaml track 의 LIVE 값 (KEEP·max 값 제외) 위에 프리셋 reference 를 덮은 것 → 같은 프리셋이면 늘 같은 기준
import os

import yaml

from amr_mpc.core.ref_config import LIVE

# 프리셋이 yaml 로 되돌리지 않는 값 (real_all 은 origin robot 이 기본 — 출발점 방식은 지금 값 그대로)
KEEP = ("origin_mode", "origin_x", "origin_y", "origin_yaw_deg", "start_tolerance_xy", "start_tolerance_yaw_deg",
        "lead_speed", "lead_yaw_rate")


def load_presets(config_dir):
    with open(os.path.join(config_dir, "pitwall_presets.yaml")) as f:
        return yaml.safe_load(f) or {}


def track_base(config_dir):
    """torque_track.yaml track 의 LIVE 값 (KEEP·max 값 제외)."""
    with open(os.path.join(config_dir, "torque_track.yaml")) as f:
        t = yaml.safe_load(f)["track"]
    out = {}
    for k in LIVE:
        if k in KEEP or k not in t or str(t[k]).strip().lower() == "max":
            continue
        out[k] = t[k]
    if "lap_speeds" in out:
        out["lap_speeds"] = [float(x) for x in (out["lap_speeds"] or [])] or [0.0]
    return out


def reference_values(config_dir, name):
    """프리셋 name 의 기준 값 전체 (track_base + reference). 없는 이름이면 KeyError."""
    p = load_presets(config_dir)[name]
    return dict(track_base(config_dir), **(p.get("reference") or {}))
