"""2026-11-21 kf 게인 튜닝 시험용 인자 — est_gains·est_noise (추정기 잡음 값), sim_noise (시뮬 센서 잡음, 시간 상관 SLAM 오차).

launch 는 띄우지 않고 _nodes() 만 불러 노드 파라미터를 본다 (test_launch_modes.py 와 같은 방식).
"""
import importlib.util
import math
import os

import numpy as np
import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument

from amr_mpc.torque.estimator import NOISE_KEYS, design_gains
from amr_mpc.torque.sim_sensors import SimSensors, parse_overrides

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), os.path.join(HERE, "..", "launch", name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope="module")
def te():
    return _load("torque_experiment.launch.py")


def _params(m, exe, **args):
    ctx = LaunchContext()
    for e in m.generate_launch_description().entities:
        if isinstance(e, DeclareLaunchArgument):
            ctx.launch_configurations[e.name] = "".join(s.perform(ctx) for s in e.default_value) if e.default_value else ""
    for k, v in args.items():
        ctx.launch_configurations[k] = str(v)
    n = [n for n in m._nodes(ctx) if getattr(n, "_Node__node_executable", None) == exe][0]
    return {"".join(getattr(s, "text", str(s)) for s in k): v for d in n._Node__parameters for k, v in d.items()}


def test_default_leaves_yaml(te):
    """인자를 안 주면 추정기에 잡음 파라미터를 넘기지 않는다 → torque_estimator.yaml 그대로 (기본 동작 안 바뀜)."""
    p = _params(te, "state_estimator_node")
    assert not set(NOISE_KEYS) & set(p)
    assert _params(te, "state_estimator_node", est_gains="yaml").keys() == p.keys()


def test_candidate_preset(te):
    p = _params(te, "state_estimator_node", est_gains="candidate_1121")
    assert p["accel_noise_density"] == 0.14 and p["accel_bias_walk"] == 0.04
    assert p["pose_noise_xy"] == 0.002 and p["pose_noise_yaw_deg"] == 0.05
    assert "gyro_noise_density" not in p                         # 묶음에 없는 키는 yaml 그대로


def test_est_noise_applies_after_preset(te):
    p = _params(te, "state_estimator_node", est_gains="candidate_1121", est_noise="pose_noise_xy=0.0025, gyro_noise_density=0.0128")
    assert p["pose_noise_xy"] == 0.0025 and p["gyro_noise_density"] == 0.0128 and p["accel_noise_density"] == 0.14


@pytest.mark.parametrize("args", [{"est_gains": "nope"}, {"est_noise": "accel=0.1"}, {"est_noise": "pose_noise_xy"},
                                  {"est_noise": "pose_noise_xy=0"}, {"sim_noise": "slam_gm=0.1"}, {"sim_noise": "slam_mode=1"}])
def test_rejects_bad_values(te, args):
    with pytest.raises(RuntimeError):
        _params(te, "state_estimator_node", **args)


def _text(v):
    """문자열 파라미터는 YAML 문서꼴 TextSubstitution 튜플로 온다 → 글자로."""
    if isinstance(v, (tuple, list)):
        v = yaml.safe_load("".join(getattr(s, "text", str(s)) for s in v))
    return "" if v is None else str(v)


def test_sim_noise_reaches_plant(te):
    p = _params(te, "sim_plant_torque_node", sim_noise="slam_gm_xy=0.004,slam_gm_tau=0.5")
    assert _text(p["sensor_override"]) == "slam_gm_xy=0.004,slam_gm_tau=0.5"
    assert _text(_params(te, "sim_plant_torque_node")["sensor_override"]) == ""


def test_real_all_passes_est_args():
    m = _load("real_all.launch.py")
    assert "est_gains" in m._PASS and "est_noise" in m._PASS


def test_candidate_gains_match_analysis():
    """README_KO.md 의 후보 게인 (실차 bag 37 개로 고른 값)."""
    Kxy, Kyaw = design_gains(0.1, 0.14, 0.04, 0.002, 0.0064, 0.0005, math.radians(0.05))
    assert np.allclose(Kxy, [0.881, 7.89, -2.183], atol=2e-3)
    assert np.allclose(Kyaw, [0.863, -0.0671], atol=1e-3)


def test_parse_overrides():
    assert parse_overrides("") == {}
    assert parse_overrides("slam_noise_xy=0.0015, slam_gm_tau=1.5") == {"slam_noise_xy": 0.0015, "slam_gm_tau": 1.5}
    with pytest.raises(ValueError):
        parse_overrides("slam_jump_once=1")                        # 숫자 키만


def _stationary_poses(cfg, n_scans, seed=3):
    s = SimSensors(cfg, seed=seed)
    X, Xd, out, t = np.zeros(3), np.zeros(3), [], 0.0
    while len(out) < n_scans:
        out += s.step(t, X, Xd)["poses"]
        t += 0.005
    P = np.array([[x, y, yaw] for _, x, y, yaw in out])
    return P - P.mean(0)                                          # 서 있는 로봇: 포즈 = (상수) + 오차


def test_gauss_markov_slam_error_statistics():
    """정상 σ 와 lag-1 상관 φ = exp(−T/τ) 가 설정대로 나온다 (white·IMU 잡음 0)."""
    cfg = dict(slam_noise_xy=0.0, slam_noise_yaw_deg=0.0, slam_gm_xy=0.004, slam_gm_yaw_deg=0.1, slam_gm_tau=0.5,
               accel_noise_still=0.0, accel_noise_moving=0.0, gyro_noise_still=0.0, gyro_noise_moving=0.0)
    e = _stationary_poses(cfg, 20000)
    phi = math.exp(-0.1 / 0.5)
    for c, sig in ((0, 0.004), (1, 0.004), (2, math.radians(0.1))):
        assert abs(e[:, c].std() / sig - 1.0) < 0.08
        assert abs(np.corrcoef(e[:-1, c], e[1:, c])[0, 1] - phi) < 0.03


def test_gauss_markov_off_is_white():
    """slam_gm_* 0 (기본) 이면 SLAM 오차는 white — lag-1 상관 ≈ 0, σ = slam_noise_xy."""
    e = _stationary_poses(dict(slam_noise_xy=0.002, accel_noise_still=0.0, gyro_noise_still=0.0), 20000)
    assert abs(e[:, 0].std() / 0.002 - 1.0) < 0.05
    assert abs(np.corrcoef(e[:-1, 0], e[1:, 0])[0, 1]) < 0.03
