#!/usr/bin/env python3
"""노면별 마찰계수 μ̂ — 표(config/surfaces.yaml) 읽기·쓰기, 여러 판을 모아 대표값 정하기, 램프 기록 분석.

ROS 에 의존하지 않는다 (launch·도구·pytest 가 같이 쓴다). 절차: amr/notes_amr/mu_surface_procedure.md

왜 노면별 상수인가
  비교군 C 는 MPC 모델이 "슬립 없음" 이고, 토크 상한 τ_μ = (r/√2)·μ̂·(m g/4)·c_s 로 그 가정을 지킨다.
  μ̂ 가 크면 미끄러지고(B 처럼), 작으면 쓸데없이 느리다 → 노면마다 재서 그 노면에서는 상수로 쓴다.

μ̂ 의 정의 (tools/mu_ramp_test.py, 시뮬 SlipPlant 와 같은 45° 롤러 모델)
  미끄러지는 동안 몸체 가속도 a → 지면이 준 힘 m·a → 최소 노름 지면 토크 τ_g = pinv((T⁻¹)ᵀ)·[m a; 0]
  → 바퀴 롤러 방향 힘 √2·|τ_g,i|/r → μ̂ = max_i (그 힘) / (m g / 4).   앞·옆이면 √2·|a|/g, 대각이면 2·|a|/g.
  미끄러지는 동안(운동 마찰)이라 출발 순간 최대값(정지 마찰)보다 작거나 같다 → 상한에 쓰기에는 보수 쪽.

대표값 규칙 (summarize)
  · 판 결과: slip (μ̂ 확정) / no_slip (최대 토크까지 안 미끄러짐 → μ̂ 하한) / unconfirmed (거짓 판정, 버림)
  · 방향마다 slip 판의 중앙값. 주 방향(기본 앞·뒤 — 옆·대각은 롤러 손실·IMU 잡음이 커서 흩어짐이 크다,
    2026-11-16 시뮬 검증)마다 min_runs 판 이상이어야 "확정", 아니면 "부족"
  · 추천 μ̂ = 주 방향 중앙값 중 가장 작은 값 (어느 방향으로도 미끄러지지 않게)
  · 옆 방향을 쟀으면 같이 보여 준다. 옆이 주 방향보다 확실히 낮으면 경고 (C 식은 방향 구분이 없다)
"""
import csv
import datetime
import math
import os

import numpy as np
import yaml

from amr_mpc.core import config

SURFACE_FILE = "surfaces.yaml"
MAIN_DIRS = ("fwd", "back")
_D = 1.0 / math.sqrt(2.0)
DIR_VEC = {"fwd": (1.0, 0.0), "back": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0),
           "fl": (_D, _D), "fr": (_D, -_D), "bl": (-_D, _D), "br": (-_D, -_D)}

_HEADER = """\
# 노면별 마찰계수 μ̂ — 비교군 C 의 토크 상한 τ_μ = (r/√2)·μ̂·(m g/4)·c_s 에 쓴다 (torque_plant.yaml surface.mu_hat 대신).
# 쓰는 법:  amrall … limit_mode:=friction surface:=<이름>      (mu_hat:=값 을 같이 주면 그 값이 이긴다)
# 고치는 법: 손으로 고치지 말고 tools/mu_surface.py summarize … --write 로 (측정 근거가 같이 남는다).
#            절차·판정 규칙: amr/notes_amr/mu_surface_procedure.md,  계산: amr_mpc/core/surfaces.py
# 항목: mu_hat (추천값, 없으면 null = 미측정 → surface:= 로 쓰면 거부), status (확정 | 부족 | 미측정),
#       dirs (방향별 판 수·중앙값·최소·최대, 슬립 없는 판의 하한), measured (날짜), method, sources (기록 파일), note
"""


# ── 표 ─────────────────────────────────────────────────────────
def load(filename=SURFACE_FILE):
    """{이름: 항목 dict}. 파일이 없으면 빈 dict."""
    try:
        p = config.path(filename)
    except FileNotFoundError:
        return {}
    with open(p) as f:
        d = yaml.safe_load(f) or {}
    return d.get("surfaces") or {}


def lookup(name, filename=SURFACE_FILE):
    """노면 이름 → μ̂ (float). 없거나 미측정이면 ValueError (launch 가 조용히 기본값으로 가지 않게)."""
    table = load(filename)
    if name not in table:
        raise ValueError(f"surface:={name} — {filename} 에 없다. 있는 노면: {', '.join(sorted(table)) or '(없음)'}")
    mu = table[name].get("mu_hat")
    if mu is None:
        raise ValueError(f"surface:={name} — μ̂ 미측정 (status {table[name].get('status')}). "
                         f"절차 amr/notes_amr/mu_surface_procedure.md 로 재거나 mu_hat:=값 으로 직접 줄 것")
    mu = float(mu)
    if not (0.0 < mu < 2.0):
        raise ValueError(f"surface:={name} — μ̂ {mu} 가 범위 밖 (0 < μ̂ < 2)")
    return mu


def save(table, path):
    """표 전체를 path 에 쓴다 (머리 주석 유지)."""
    with open(path, "w", encoding="utf-8") as f:
        f.write(_HEADER)
        yaml.safe_dump({"surfaces": table}, f, allow_unicode=True, sort_keys=True, default_flow_style=False)


# ── μ̂ 계산 ─────────────────────────────────────────────────────
def mu_from_accel(a_vec, plant):
    """몸체 가속도 a (몸체 좌표 2D) → μ̂ (45° 롤러 모델). tools/mu_ramp_test.py 와 같은 식."""
    F = np.array([plant.mass * a_vec[0], plant.mass * a_vec[1], 0.0])
    tau_g = np.linalg.pinv(plant.Tinv.T) @ F
    mu_i = math.sqrt(2.0) * np.abs(tau_g) / plant.r / (plant.mass * plant.g / 4.0)
    return float(mu_i.max())


def _slope(t, v):
    if len(t) < 3:
        return float("nan")
    return float(np.polyfit(np.asarray(t) - t[0], v, 1)[0])


def analyze_ramp_node_csv(path, plant, slip=0.12, slip_frac=0.15, persist_s=0.05, window_s=0.3, gap_grow=0.05):
    """torque_ramp_node(controller:=ramp) 한 판 CSV → 판 결과 dict.

    이 노드는 슬립 뒤에도 토크를 계속 올린다(유지 구간이 없다). 그래서 mu_ramp_test 의 판정을 기록에 적용한다:
      슬립 시작 = v_wheel − v_est > slip + slip_frac·|v_wheel| 가 persist_s 동안 이어진 첫 시각
      그 뒤 window_s 동안 바퀴–몸체 차가 gap_grow 넘게 더 벌어지면 slip, 그 구간 몸체 가속(v_est 기울기)으로 μ̂.
      미끄러지는 동안 몸체 가속은 토크가 아니라 마찰이 정한다 → 토크가 계속 올라가도 같은 식을 쓴다.
    v_est = 추정기(라이다 + IMU) 몸체 속도. 바퀴와 독립. 방향은 파일 이름(ramp_fwd_…, ramp_left_…)에서.
    """
    name = os.path.basename(path)
    direction = next((k for k in DIR_VEC if f"_{k}_" in name), None)
    if direction is None:
        raise ValueError(f"{name}: 파일 이름에서 방향(fwd·back·left·right…)을 못 찾았다")
    u = np.array(DIR_VEC[direction])
    rows = list(csv.DictReader(open(path)))

    def col(k):
        return np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
    t, vw, ve = col("t"), col("v_wheel_dir"), col("v_est_dir")
    tau = col("tau_g") if "tau_g" in rows[0] else np.full(len(t), np.nan)
    ok = np.isfinite(t) & np.isfinite(vw) & np.isfinite(ve)
    t, vw, ve, tau = t[ok], vw[ok], ve[ok], tau[ok]
    gap = vw - ve
    out = {"source": name, "dir": direction, "mu": float("nan"), "tau_slip": float("nan"), "t_slip": float("nan"),
           "v_slip": float("nan"), "gap_grow": float("nan")}
    bad = gap > slip + slip_frac * np.abs(vw)
    k0 = None
    for i in range(len(t)):
        if bad[i]:
            j = i
            while j < len(t) and bad[j] and t[j] - t[i] < persist_s:
                j += 1
            if j < len(t) and bad[j]:
                k0 = j
                break
    if k0 is None:
        win = t >= t[-1] - 0.15
        a = _slope(t[win], ve[win])
        out["result"] = "no_slip"
        out["mu"] = mu_from_accel(a * u, plant) if np.isfinite(a) else float("nan")
        out["tau_slip"] = float(np.nanmax(tau)) if np.isfinite(tau).any() else float("nan")
        return out
    idx = np.where((t >= t[k0]) & (t <= t[k0] + window_s))[0]
    out.update(t_slip=float(t[k0] - t[0]), v_slip=float(vw[k0]), tau_slip=float(tau[k0]),
               gap_grow=float(gap[idx[-1]] - gap[idx[0]]))
    if len(idx) < 10 or out["gap_grow"] <= gap_grow:
        out["result"] = "unconfirmed"
        return out
    a = _slope(t[idx], ve[idx])
    out["result"] = "slip"
    out["mu"] = mu_from_accel(a * u, plant)
    return out


def analyze_sliding_csv(path, plant, gap_min=0.30, gap_frac=0.20, min_n=8, min_r2=0.9):
    """torque_step_node(step_…csv)·torque_ramp_node(ramp_…csv) 한 판 → 판 결과 dict (2026-11-18).

    바퀴가 몸체보다 확실히 빨리 도는 구간 — v_wheel − v_est > gap_min 이고 > gap_frac·|v_wheel| — 중 가장 긴 연속 구간의
    몸체 속도 기울기 a 로 μ̂ (운동 마찰). 스텝은 처음부터 네 바퀴가 헛돌아 바퀴가 속도 한계(≈ 1.6 m/s)에 먼저 닿고 몸체가
    마찰로 따라붙는다 → 차이가 줄어드는 쪽이라 analyze_ramp_node_csv 의 "더 벌어짐" 확인으로는 늘 버려진다. 그래서 따로 둔다.
    구간이 min_n 표본보다 짧거나 직선 맞춤 R² < min_r2 면 unconfirmed. 슬립이 없는 판은 이 방법으로 하한을 내지 않는다.
    """
    name = os.path.basename(path)
    direction = next((k for k in DIR_VEC if f"_{k}_" in name), None)
    if direction is None:
        raise ValueError(f"{name}: 파일 이름에서 방향(fwd·back·left·right…)을 못 찾았다")
    u = np.array(DIR_VEC[direction])
    rows = list(csv.DictReader(open(path)))

    def col(k):
        return np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
    t, vw, ve = col("t"), col("v_wheel_dir"), col("v_est_dir")
    tau = col("tau_g") if "tau_g" in rows[0] else np.full(len(t), np.nan)
    ok = np.isfinite(t) & np.isfinite(vw) & np.isfinite(ve)
    t, vw, ve, tau = t[ok], vw[ok], ve[ok], tau[ok]
    gap = vw - ve
    out = {"source": name, "dir": direction, "result": "unconfirmed", "mu": float("nan"), "tau_slip": float("nan"),
           "t_slip": float("nan"), "v_slip": float("nan"), "gap_grow": float("nan"), "r2": float("nan"), "n": 0}
    s = (gap > gap_min) & (gap > gap_frac * np.abs(vw)) & (t > t[0] + 0.02)
    idx = np.flatnonzero(s)
    if len(idx) == 0:
        return out
    blocks = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    b = max(blocks, key=len)
    out.update(n=int(len(b)), t_slip=float(t[b[0]] - t[0]), v_slip=float(vw[b[0]]), tau_slip=float(tau[b[0]]),
               gap_grow=float(gap[b[-1]] - gap[b[0]]))
    if len(b) < min_n:
        return out
    tt, vv = t[b] - t[b[0]], ve[b]
    k, c0 = np.polyfit(tt, vv, 1)
    ss = float(np.sum((vv - vv.mean()) ** 2))
    out["r2"] = 1.0 - float(np.sum((vv - (k * tt + c0)) ** 2)) / ss if ss > 0 else float("nan")
    if not (out["r2"] >= min_r2) or k <= 0:
        return out
    out["result"] = "slip"
    out["mu"] = mu_from_accel(float(k) * u, plant)
    return out


def read_mu_ramp_summary(path):
    """tools/mu_ramp_test.py --csv … 의 _summary.csv → 판 결과 dict 목록."""
    out = []
    for r in csv.DictReader(open(path)):
        mu = float(r["mu"]) if r["mu"] not in ("", "nan") else float("nan")
        out.append({"source": os.path.basename(path), "dir": r["dir"], "result": r["result"], "mu": mu,
                    "tau_slip": float(r["amp_slip"]) if r.get("amp_slip") not in (None, "", "nan") else float("nan")})
    return out


# ── 대표값 ─────────────────────────────────────────────────────
def summarize(runs, main_dirs=MAIN_DIRS, min_runs=5, lateral_warn=0.85):
    """판 결과 목록 → (방향별 통계 dict, 추천 dict).

    추천 dict: mu_hat (float 또는 None), status ('확정' | '부족'), reason, warnings (list)
    """
    dirs = {}
    for r in runs:
        d = dirs.setdefault(r["dir"], {"slip": [], "lower": [], "unconfirmed": 0})
        if r["result"] == "slip" and np.isfinite(r["mu"]):
            d["slip"].append(float(r["mu"]))
        elif r["result"] == "no_slip" and np.isfinite(r["mu"]):
            d["lower"].append(float(r["mu"]))
        else:
            d["unconfirmed"] += 1
    stats = {}
    for k, d in dirs.items():
        s = np.array(d["slip"])
        stats[k] = {"n": int(len(s)),
                    "median": round(float(np.median(s)), 3) if len(s) else None,
                    "min": round(float(s.min()), 3) if len(s) else None,
                    "max": round(float(s.max()), 3) if len(s) else None,
                    "lower_bound": round(float(max(d["lower"])), 3) if d["lower"] else None,
                    "unconfirmed": int(d["unconfirmed"])}
    warnings = []
    missing = [k for k in main_dirs if k not in stats or stats[k]["n"] < min_runs]
    meds = [stats[k]["median"] for k in main_dirs if k in stats and stats[k]["median"] is not None]
    mu_hat = round(min(meds), 3) if meds else None
    if missing:
        status = "부족"
        reason = (f"주 방향 {', '.join(main_dirs)} 마다 슬립 확인 {min_runs} 판 이상 필요 — 부족: "
                  + ", ".join(f"{k} {stats[k]['n'] if k in stats else 0} 판" for k in missing))
    else:
        status = "확정"
        reason = f"주 방향 {', '.join(main_dirs)} 중앙값 중 가장 작은 값"
    for k, s in stats.items():
        if mu_hat is not None and s["lower_bound"] is not None and s["lower_bound"] > mu_hat * 1.1:
            warnings.append(f"{k}: 슬립 없는 판의 하한 {s['lower_bound']} > 추천 {mu_hat} — 판정·토크 범위 확인")
        if mu_hat is not None and k not in main_dirs and s["median"] is not None and s["median"] < lateral_warn * mu_hat:
            warnings.append(f"{k}: 중앙값 {s['median']} 가 추천 {mu_hat} 의 {lateral_warn:.0%} 미만 — 그 방향은 상한보다 먼저 "
                            f"미끄러질 수 있다 (옆걸음 판정은 롤러 손실로 거짓 슬립이 섞이기 쉬우니 판 수를 늘려 확인)")
        if s["n"] >= 3 and s["median"]:
            spread = (s["max"] - s["min"]) / s["median"]
            if spread > 0.4:
                warnings.append(f"{k}: 흩어짐 (최대−최소)/중앙 {spread:.0%} — 바닥 상태·배터리·판정 확인")
    return stats, {"mu_hat": mu_hat, "status": status, "reason": reason, "warnings": warnings}


def make_entry(stats, rec, sources, method, note="", date=None):
    """표에 넣을 항목 dict."""
    return {"mu_hat": rec["mu_hat"], "status": rec["status"], "dirs": stats,
            "measured": date or datetime.date.today().isoformat(), "method": method,
            "sources": sorted(set(sources)), "note": note or rec["reason"]}


# ── 바퀴별 마찰 상한 배율 (2026-11-18) ─────────────────────────────────────────────
#   τ̄_i = (r/√2)·μ̂·N_i·c_s / g_i = τ_μ · s_i,   s_i = (N_i / N̄) / g_i
#   N_i = 바퀴 i 에 실린 무게 (저울), g_i = 실제 토크 / 명령 (모터 차이, 모르면 1).
#   μ̂ 는 노면 값(위 표), s_i 는 차체 값 (torque_plant.yaml surface.wheel_limit_scale).
#   바퀴별 램프 슬립 시작(wheel_slip_onsets)은 검산용 — 같은 토크에서 가벼운 바퀴가 먼저 미끄러져야 한다.
WHEELS = ("FL", "FR", "RL", "RR")


def wheel_slip_onsets(path, plant, gap=0.10, gap_frac=0.15, persist=5):
    """torque_ramp_node CSV 한 판 → 바퀴마다 먼저 헛돌기 시작한 시각과 그때 모터 보고 토크(없으면 None).
    바퀴 표면 속도 r·ω_i 가 몸체 운동으로 본 무슬립 속도 r·(T⁻¹v)_i 보다 gap + gap_frac·|r ω_i| 넘게 persist 표본 이어지면 시작."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    col = lambda k: np.array([float(r[k]) for r in rows])
    t = col("t")
    om = np.vstack([col(f"omega_{w}") for w in WHEELS]).T
    fb = np.vstack([col(f"tau_fb_{w}") for w in WHEELS]).T
    v = np.vstack([col("vx_body"), col("vy_body"), col("wz")]).T
    c = (plant.Tinv @ v.T).T
    lead = plant.r * (om - c) * np.sign(om)
    out = {}
    for i, w in enumerate(WHEELS):
        bad = lead[:, i] > gap + gap_frac * np.abs(plant.r * om[:, i])
        j = next((j for j in range(len(t) - persist + 1) if bad[j:j + persist].all()), None)
        out[w] = None if j is None else {"t": float(t[j]), "tau_fb": float(abs(fb[j, i]))}
    return out


def plane_loads(total, front, left):
    """저울 1 개로 잰 쌍 합 → 바퀴별 하중 [FL, FR, RL, RR] (평평한 바닥, 차체 비틀림 없음 가정).
    total = 전체, front = 앞 두 바퀴 합, left = 왼쪽 두 바퀴 합 (같은 단위).
    네 바퀴 하중은 N = A + B·(앞 +1/뒤 −1) + C·(왼 +1/오 −1) + D·(앞뒤 × 좌우) 꼴인데, 무게중심이 정하는 A·B·C 는 쌍 합으로
    정해지고 받침 높이 몇 mm 에 거의 안 흔들린다. 대각 성분 D 는 받침·바닥 높이에 민감해서 저울로는 못 정한다 → D = 0 으로 두고
    실제 바닥의 대각 쏠림은 바퀴별 램프 슬립 시작으로 본다."""
    total, front, left = float(total), float(front), float(left)
    if not (total > 0 and 0 < front < total and 0 < left < total):
        raise ValueError("0 < 앞 합, 왼쪽 합 < 전체 여야 한다")
    A = total / 4.0
    B = (front - total / 2.0) / 2.0
    C = (left - total / 2.0) / 2.0
    return [A + B + C, A + B - C, A - B + C, A - B - C]


def wheel_scale(loads=None, onsets=None, motor_gain=None):
    """바퀴별 배율 추천.
    loads: [FL, FR, RL, RR] 무게 (단위 상관없음) → s_i = (N_i/N̄)/g_i  (주 방법)
    onsets: wheel_slip_onsets 결과 목록 → 바퀴마다 슬립 시작 토크 중앙값, 슬립한 판 수 (하중이 없을 때는 이것으로 — 낮추기만)
    반환 {"scale": [4], "method": str, "onset": {바퀴: (판 수, 중앙 토크|None)}, "warnings": [...]}"""
    g = np.ones(4) if motor_gain is None else np.asarray(motor_gain, float)
    if g.shape != (4,) or not (g > 0).all():
        raise ValueError("motor_gain 은 바퀴 4 개, 양수")
    warnings = []
    onset = {}
    for w in WHEELS:
        vals = [o[w]["tau_fb"] for o in (onsets or []) if o.get(w)]
        onset[w] = (len(vals), float(np.median(vals)) if vals else None)
    if loads is not None:
        N = np.asarray(loads, float)
        if N.shape != (4,) or not (N > 0).all():
            raise ValueError("loads 는 바퀴 4 개, 양수 [FL, FR, RL, RR]")
        s = (N / N.mean()) / g
        method = "하중" + (" + 모터 비" if motor_gain is not None else "")
        if onsets:
            first = min(WHEELS, key=lambda w: -onset[w][0])          # 가장 자주 미끄러진 바퀴
            light = WHEELS[int(np.argmin(s))]
            if first != light:
                warnings.append(f"하중으로는 {light} 가 가장 약한데 램프에서는 {first} 가 가장 자주 미끄러졌다 — "
                                "바닥·차체의 대각 쏠림(저울로는 안 보임)이나 롤러·모터 차이 의심. 램프 판을 늘려 확인")
    elif onsets:
        known = {w: m for w, (n, m) in onset.items() if m is not None}
        if len(known) < 2:
            raise ValueError("슬립 시작이 두 바퀴 이상에서 잡혀야 비교할 수 있다 (판을 늘리거나 하중을 잴 것)")
        ref = float(np.median(list(known.values())))
        s = np.array([min(1.0, known[w] / ref) if w in known else 1.0 for w in WHEELS]) / g
        method = "램프 슬립 시작 (낮추기만)" + (" + 모터 비" if motor_gain is not None else "")
        warnings.append("하중 없이 슬립 시작 토크로만 낸 값 — 판정 지연이 섞인다. 저울로 바퀴별 하중을 재면 그쪽을 쓴다")
    else:
        raise ValueError("loads 나 onsets 중 하나는 줘야 한다")
    if (s > 1.5).any() or (s <= 0).any():
        raise ValueError(f"배율 {np.round(s, 3)} 가 0~1.5 밖 — 입력 확인")
    if (np.abs(s - 1.0) > 0.25).any():
        warnings.append("배율이 1 에서 25 % 넘게 벗어난다 — 무게 배치(배터리·케이블)나 측정을 다시 확인")
    return {"scale": [round(float(x), 3) for x in s], "method": method, "onset": onset, "warnings": warnings}
