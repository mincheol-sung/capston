# 기준 경로 설정 → 경로 객체 + 부하 검사 (2026-11-28 pitwall, ROS 없이 시험 — test/test_ref_config.py)
# ---------------------------------------------------------------
# figure8_reference_node 가 기동 때 한 번, 그리고 실행 중 parameter 가 바뀔 때마다 부른다.
#   build(cfg, v_limit, budget, k_geom, path_follow) → (path, 주행 부하, 정렬 회전 부하)
#   검사에 걸리면 ValueError (메시지 = 기동 때 SystemExit 로 내던 문장 그대로).
# LIVE: 실행 중에 바꿀 수 있는 parameter. 나머지는 launch 때만 (노드가 read_only 로 선언).
#   빼는 이유 — dt·horizon_steps·rate_hz: MPC 기준열 격자 / load_budget·max_wheel_rad_per_sec·abort_error: 안전 문턱 /
#   path_follow_lead_s: launch 가 rate_hz 50 과 같이 정한다 / topic·frame·wait_for_arm: 배선
# 적용 시점 (pitwall_design.md 사용자 결정): 바꾼 값은 "다음 판" 부터. 진행 중인 판의 경로는 바꾸지 않는다.
# ---------------------------------------------------------------
import math

from amr_mpc.core.figure8 import Figure8
from amr_mpc.core.polyline import Polyline
from amr_mpc.core.segment import Segment
from amr_mpc.core.square import Square
from amr_mpc.core.zigzag import Zigzag

LIVE = (
    "shape", "width", "height", "lap_speeds", "speed", "accel", "laps", "yaw_mode", "spin_rate", "align_s",
    "speed_mod_amp", "speed_mod_cycles", "speed_mod_phase_deg", "speed_mod_shape", "crossing_blend_m", "first_turn",
    "start_delay", "hold_s", "resume_ramp_s", "resync_window_s",
    "origin_mode", "origin_x", "origin_y", "origin_yaw_deg", "start_tolerance_xy", "start_tolerance_yaw_deg",
    "lead_speed", "lead_yaw_rate",
    "seg_distance", "seg_dir_deg", "seg_angle_deg", "seg_yaw_rate", "seg_yaw_accel", "seg_out_and_back", "seg_pause_s",
    "square_side", "square_corner_speed", "square_v_max", "square_yaw_blend", "square_corner_mode", "square_rot_s",
    "square_dwell_s", "poly_path",
    "zigzag_n", "zigzag_length", "zigzag_angle_deg", "zigzag_profile", "zigzag_rot_s", "zigzag_dwell_s", "zigzag_corner",
    "zigzag_corner_speed", "zigzag_turn_m", "zigzag_reverse",
)

# rqt_reconfigure 슬라이더 범위 (구현 선택 — 실차에서 쓴 값보다 넓게). 부하 예산이 실제 한계다. step 0 = 제약 없음
#   (rclpy 는 step 의 배수가 아니면 거부한다 — circ speed 1.232 같은 값을 받으려면 0)
UI_RANGES = {
    "speed": (0.0, 2.0), "accel": (0.05, 3.0), "width": (0.3, 6.0), "height": (0.1, 4.0), "laps": (0.25, 10.0),
    "align_s": (0.3, 5.0), "start_delay": (0.0, 10.0), "hold_s": (-1.0, 30.0), "spin_rate": (0.1, 3.0),
    "speed_mod_amp": (0.0, 0.9), "crossing_blend_m": (0.0, 2.0),
    "square_side": (0.2, 5.0), "square_corner_speed": (0.0, 1.5), "square_v_max": (0.0, 2.0),
    "square_yaw_blend": (0.0, 2.0), "square_rot_s": (0.2, 5.0), "square_dwell_s": (0.0, 3.0),
    "seg_distance": (0.05, 5.0), "zigzag_n": (1, 20), "zigzag_length": (0.2, 5.0), "zigzag_angle_deg": (-170.0, 170.0),
    "zigzag_corner_speed": (0.0, 1.5), "zigzag_turn_m": (0.0, 2.5), "zigzag_rot_s": (0.1, 5.0),
}
UI_NOTES = {"square_v_max": "0 = 부하 예산 최고 (load_budget·V_LIMIT)", "hold_s": "음수 = 끝까지 유지",
            "speed": "zigzag 에서는 직선 최고 속도 상한, 0 = 부하 예산 최고", "poly_path": "R1,L1,… = 오른쪽/왼쪽 90° 돌고 그 길이 [m]",
            "shape": "figure8 | circles8 | line | rotate | square | zigzag | polyline", "yaw_mode": "tangent | fixed | spin",
            "first_turn": "figure8·circles8 첫 회전 방향 left | right (right = 출발 헤딩 축에 대해 거울, 오른쪽 원부터)"}

CHOICES = {"shape": ("figure8", "circles8", "line", "rotate", "square", "zigzag", "polyline"),
           "yaw_mode": ("tangent", "fixed", "spin"), "origin_mode": ("robot", "fixed"),
           "square_corner_mode": ("pass", "stop"), "speed_mod_shape": ("sin", "square"), "first_turn": ("left", "right"),
           "zigzag_profile": ("sine", "square"), "zigzag_corner": ("stop", "pass")}

# 판 요약·비교에 쓰는 모양별 값 (나머지 LIVE 값은 그 모양에서 안 쓰인다)
_COMMON = ("yaw_mode", "start_delay", "hold_s")
RELEVANT = {
    "figure8": ("width", "height", "speed", "accel", "laps", "lap_speeds", "align_s", "speed_mod_amp", "speed_mod_cycles",
                "speed_mod_phase_deg", "speed_mod_shape", "spin_rate", "first_turn"),
    "circles8": ("width", "speed", "accel", "laps", "lap_speeds", "align_s", "crossing_blend_m", "speed_mod_amp",
                 "speed_mod_cycles", "speed_mod_phase_deg", "speed_mod_shape", "spin_rate", "first_turn"),
    "line": ("seg_distance", "seg_dir_deg", "speed", "accel", "seg_out_and_back", "seg_pause_s"),
    "rotate": ("seg_angle_deg", "seg_yaw_rate", "seg_yaw_accel", "seg_out_and_back", "seg_pause_s"),
    "square": ("square_side", "square_corner_speed", "square_v_max", "square_yaw_blend", "square_corner_mode",
               "square_rot_s", "square_dwell_s", "accel", "laps", "align_s"),
    "polyline": ("poly_path", "square_corner_speed", "square_v_max", "square_yaw_blend", "square_corner_mode",
                 "square_rot_s", "square_dwell_s", "accel", "align_s"),
    "zigzag": ("zigzag_n", "zigzag_length", "zigzag_angle_deg", "zigzag_profile", "zigzag_rot_s", "zigzag_dwell_s",
               "zigzag_corner", "zigzag_corner_speed", "zigzag_turn_m", "zigzag_reverse", "speed", "accel", "spin_rate",
               "align_s"),
}


def relevant(cfg):
    """그 모양에서 실제로 쓰이는 값만 (요약·판 비교용)."""
    shape = str(cfg["shape"]).strip().lower()
    return {k: cfg[k] for k in RELEVANT.get(shape, ()) + _COMMON if k in cfg}


def check_choices(cfg):
    for k, allowed in CHOICES.items():
        if k in cfg and str(cfg[k]).strip().lower() not in allowed:
            raise ValueError(f"{k}={cfg[k]!r} — 허용값 {' | '.join(allowed)}")
    if str(cfg["shape"]).strip().lower() in ("square", "polyline") and str(cfg["yaw_mode"]).strip().lower() == "spin":
        raise ValueError(f"shape {cfg['shape']} 는 yaw_mode tangent | fixed (spin 은 figure8·circles8·zigzag 만)")


def make_path(cfg, v_limit, budget):
    """cfg (parameter 이름 → 값) → 경로 객체. 값이 틀리면 ValueError (경로 클래스가 던지는 문장)."""
    g = cfg.__getitem__
    shape = str(g("shape")).strip().lower()
    if shape in ("line", "rotate"):
        return Segment(shape=shape, distance=g("seg_distance"), dir_deg=g("seg_dir_deg"),
                       speed=g("speed"), accel=g("accel"), angle_deg=g("seg_angle_deg"),
                       yaw_rate=g("seg_yaw_rate"), yaw_accel=g("seg_yaw_accel"),
                       out_and_back=bool(g("seg_out_and_back")), pause_s=g("seg_pause_s"))
    if shape == "square":
        vmax = float(g("square_v_max"))
        return Square(side=g("square_side"), corner_speed=g("square_corner_speed"), accel=g("accel"),
                      v_max=vmax if vmax > 0.0 else budget * v_limit, laps=g("laps"),
                      yaw_mode=str(g("yaw_mode")), yaw_blend=g("square_yaw_blend"), align_s=g("align_s"),
                      corner_mode=str(g("square_corner_mode")).strip().lower(), rot_s=g("square_rot_s"),
                      dwell_s=g("square_dwell_s"))
    if shape == "polyline":                       # 2026-11-25 꺾은선 (core/polyline.py) — square_* 값을 같이 쓴다
        vmax = float(g("square_v_max"))
        return Polyline(path=str(g("poly_path")), corner_speed=g("square_corner_speed"), accel=g("accel"),
                        v_max=vmax if vmax > 0.0 else budget * v_limit, yaw_mode=str(g("yaw_mode")),
                        yaw_blend=g("square_yaw_blend"), align_s=g("align_s"),
                        corner_mode=str(g("square_corner_mode")).strip().lower(), rot_s=g("square_rot_s"),
                        dwell_s=g("square_dwell_s"))
    if shape == "zigzag":
        vmax = float(g("speed"))                  # 지그재그에서 speed = 직선 최고 속도 상한 (launch: max → 0)
        return Zigzag(n=g("zigzag_n"), length=g("zigzag_length"), angle_deg=g("zigzag_angle_deg"), accel=g("accel"),
                      v_max=vmax if vmax > 0.0 else budget * v_limit,
                      profile=str(g("zigzag_profile")).strip().lower(), rot_s=g("zigzag_rot_s"),
                      dwell_s=g("zigzag_dwell_s"), yaw_mode=str(g("yaw_mode")),
                      spin_rate=float(g("spin_rate")), align_s=float(g("align_s")), reverse=bool(g("zigzag_reverse")),
                      corner_mode=str(g("zigzag_corner")).strip().lower(), corner_speed=float(g("zigzag_corner_speed")),
                      turn_m=float(g("zigzag_turn_m")))
    if shape not in ("figure8", "circles8"):
        raise ValueError(f"shape={shape!r} — figure8 | circles8 | line | rotate | square | zigzag | polyline")
    return Figure8(width=g("width"), height=g("height"), speed=g("speed"),
                   accel=g("accel"), laps=g("laps"), yaw_mode=str(g("yaw_mode")),
                   align_s=g("align_s"),
                   lap_speeds=[v for v in g("lap_speeds") if v > 0.0] or None,
                   curve="circles" if shape == "circles8" else "gerono",
                   crossing_blend=float(g("crossing_blend_m")), spin_rate=float(g("spin_rate")),
                   speed_mod_amp=float(g("speed_mod_amp")), speed_mod_cycles=float(g("speed_mod_cycles")),
                   speed_mod_phase_deg=float(g("speed_mod_phase_deg")),
                   speed_mod_shape=str(g("speed_mod_shape")), first_turn=str(g("first_turn")))


def check_load(path, cfg, v_limit, budget, k_geom):
    """기준 부하 |vx|+|vy|+K_GEOM·|ω| ≤ load_budget·V_LIMIT (fake_reference_node 와 같은 규칙). (주행, 정렬) 부하."""
    shape = str(cfg["shape"]).strip().lower()
    run, spin = path.peak_load(k_geom)
    cap = budget * v_limit
    if run > cap and shape == "zigzag":
        raise ValueError(
            f"지그재그 기준 부하 {run:.3f} > 예산 {cap:.3f} ({budget*100:.0f} % of {v_limit:.2f}). "
            f"speed (직선 최고 속도 상한) 를 낮추거나 zigzag_rot_s 를 늘려라.")
    if run > cap and shape in ("square", "polyline"):
        raise ValueError(
            f"정사각형 기준 부하 {run:.3f} > 예산 {cap:.3f} ({budget*100:.0f} % of {v_limit:.2f}). "
            f"square_v_max 를 낮추거나 (tangent 면) square_yaw_blend 를 늘려라 — 꼭짓점 헤딩 회전이 부하에 더해진다.")
    if run > cap:                                 # 주행 부하는 speed 에 선형
        raise ValueError(
            f"기준 부하 {run:.3f} > 예산 {cap:.3f} ({budget*100:.0f} % of {v_limit:.2f}). "
            f"speed 를 {float(cfg['speed']) * cap / run:.3f} m/s 이하로 낮추거나 height 를 줄여라 "
            f"(최대곡률 {path.max_curvature():.2f} 1/m 이 ω=κv 를 키운다).")
    if spin > cap:                                # 정렬 회전 부하는 align_s 에 반비례
        raise ValueError(f"정렬 회전 부하 {spin:.3f} > 예산 {cap:.3f}. "
                         f"align_s 를 {float(cfg['align_s']) * spin / cap:.2f} s 이상으로 늘려라.")
    return run, spin


def build(cfg, v_limit, budget, k_geom, path_follow=False):
    """검사 전부: 허용값 → 경로 생성 → 부하 예산 → path following 가능 모양. (path, run, spin) 또는 ValueError."""
    check_choices(cfg)
    path = make_path(cfg, v_limit, budget)
    run, spin = check_load(path, cfg, v_limit, budget, k_geom)
    shape = str(cfg["shape"]).strip().lower()
    if path_follow and shape not in ("figure8", "circles8", "square", "zigzag"):
        raise ValueError(f"path_follow_lead_s 는 shape figure8 | circles8 | square | zigzag 만 (받은 {shape})")
    return path, run, spin


def _fmt(v):
    if isinstance(v, float):
        return f"{v:g}"
    if isinstance(v, (list, tuple)):
        return "[" + ",".join(_fmt(x) for x in v) + "]"
    return str(v)


def summary(cfg, path, run, v_limit):
    """한 줄 요약: 모양 · 쓰이는 값 · 기준 시간 · 부하 % (RViz 글자·로그·pit show)."""
    shape = str(cfg["shape"]).strip().lower()
    skip = set()
    if not float(cfg.get("speed_mod_amp", 0.0)) > 0.0:
        skip |= {"speed_mod_amp", "speed_mod_cycles", "speed_mod_phase_deg", "speed_mod_shape"}
    if str(cfg["yaw_mode"]).strip().lower() != "spin":
        skip.add("spin_rate")
    if not any(x > 0 for x in cfg.get("lap_speeds", [0.0])):
        skip.add("lap_speeds")
    keys = [k for k in RELEVANT.get(shape, ()) if k in cfg and k not in skip]
    vals = " ".join(f"{k}={_fmt(cfg[k])}" for k in keys)
    T = float(getattr(path, "T_total", float("nan")))
    T = T if math.isfinite(T) else float("nan")
    return f"{shape} {vals} yaw={cfg['yaw_mode']} | 기준 {T:.1f} s | 부하 {run / v_limit * 100:.0f} %"
