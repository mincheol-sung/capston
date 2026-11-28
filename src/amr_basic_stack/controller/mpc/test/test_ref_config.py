# pytest -q controller/mpc/test/test_ref_config.py   (ROS 불필요) — 2026-11-28 pitwall 실시간 기준 변경 검사
import pytest
import yaml
import os

from amr_mpc.body import mpc as mpc_mod
from amr_mpc.core import ref_config

CFG_DIR = os.path.join(os.path.dirname(__file__), "..", "config")


def base_cfg(**kw):
    """torque_track.yaml track 의 LIVE 값 (launch 가 노드에 넘기는 것과 같은 타입)."""
    with open(os.path.join(CFG_DIR, "torque_track.yaml")) as f:
        t = yaml.safe_load(f)["track"]
    t["lap_speeds"] = [float(x) for x in (t.get("lap_speeds") or [])] or [0.0]
    if t["zigzag_corner_speed"] == "max":
        t["zigzag_corner_speed"] = 0.3
    t["resume_ramp_s"], t["resync_window_s"] = -1.0, 2.0
    t["zigzag_n"] = int(t["zigzag_n"])
    cfg = {k: t[k] for k in ref_config.LIVE}
    cfg.update(kw)
    return cfg


V, B, K = 0.04 * 38.746, 1.0, mpc_mod.K_GEOM


def test_poly_alias_values_pass_budget():
    """poly 별칭 (polyline, accel 1.5, square_v_max 1.5, 통과) 은 예산 안 — 실제로 실차에서 돈 판 조건."""
    cfg = base_cfg(shape="polyline", accel=1.5, square_v_max=1.5, square_corner_mode="pass")
    path, run, spin = ref_config.build(cfg, V, B, K)
    assert run <= B * V
    assert "polyline" in ref_config.summary(cfg, path, run, V)


def test_circ_alias_speed_is_at_budget_and_above_is_rejected():
    """circ 별칭 1.232 m/s 는 통과, 1.5 m/s 는 부하 예산에 걸린다 (amr_aliases.sh circ 주석과 같은 결론)."""
    ok = base_cfg(shape="circles8", width=3.0, speed=1.232, accel=1.5, laps=3.0)
    ref_config.build(ok, V, B, K)
    with pytest.raises(ValueError, match="기준 부하"):
        ref_config.build(dict(ok, speed=1.5), V, B, K)


def test_bad_choice_and_bad_poly_path_are_rejected():
    with pytest.raises(ValueError, match="yaw_mode"):
        ref_config.build(base_cfg(yaw_mode="sideways"), V, B, K)
    with pytest.raises(ValueError):
        ref_config.build(base_cfg(shape="polyline", poly_path="X9"), V, B, K)
    with pytest.raises(ValueError, match="spin"):
        ref_config.build(base_cfg(shape="square", yaw_mode="spin"), V, B, K)


def test_relevant_keeps_only_values_used_by_shape():
    rel = ref_config.relevant(base_cfg(shape="circles8"))
    assert "width" in rel and "speed" in rel
    assert "poly_path" not in rel and "square_side" not in rel and "height" not in rel


def test_path_follow_only_for_supported_shapes():
    with pytest.raises(ValueError, match="path_follow"):
        ref_config.build(base_cfg(shape="polyline", accel=1.0, square_v_max=1.0), V, B, K, path_follow=True)
