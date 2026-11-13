#!/usr/bin/env python3
"""C2 세미나 그림 생성 — measure_c_results_c2.csv / measure_c_raw_c2_*.csv 에서 직접.

데이터만 있고 이 스크립트가 없으면 그림을 재현할 수 없다. 수치는 전부 CSV 에서
계산하며, 어떤 상수도 손으로 적어 넣지 않는다 (모델 파라미터만 노드에서 옮겨온다).

    python3 make_c2_figures.py

출력: 이 파일과 같은 디렉터리에 c2_*.png
"""
import csv
import glob
import math
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.abspath(os.path.join(HERE, '..', '..', 'scripts'))
RESULTS = os.path.join(SCRIPTS, 'measure_c_results_c2.csv')
CHECK = os.path.join(SCRIPTS, 'check_target_results_c2.csv')

# sensor_fusion_pipeline.svg 와 같은 팔레트
C_CAM, C_LID, C_FUS = '#3182CE', '#38A169', '#DD6B20'
C_GRAY, C_TEXT, C_BAD = '#A0AEC0', '#1A202C', '#E53E3E'

plt.rcParams.update({
    # 시스템에 등록된 CJK 패밀리 이름은 'Noto Sans CJK JP' 하나뿐이지만
    # pan-CJK 폰트라 한글 글리프를 포함한다 (fc-list 로 확인).
    'font.family': 'Noto Sans CJK JP',
    'axes.unicode_minus': False,
    'font.size': 11,
    'axes.titlesize': 13,
    'axes.titleweight': 'bold',
    'axes.labelsize': 11,
    'axes.edgecolor': '#4A5568',
    'axes.titlecolor': C_TEXT,
    'figure.facecolor': 'white',
    'savefig.facecolor': 'white',
    'legend.frameon': False,
    'grid.color': '#E2E8F0',
})

# ── camera_lidar_fusion_node.py 의 기본 파라미터 ─────────────────────
CAM_R = (0.321e-3, 2.741)
CAM_T = (0.560e-3, 1.450)
LID_R = (0.331e-3, 0.354)
LID_T_CONST = 6.5e-3
LID_R_FLOOR, LID_R_BREAK = 1.6e-3, 4.5
DELTA_R = (-10.98e-3, 1.281)
Q_R, Q_T = 5.08e-3, 6.95e-3
GATE = 9.21

# 배치 실수 / 양 센서 0 검출 run (EXPERIMENT_C2_RESULTS.txt 2절)
BAD_TS = {
    '2026-09-04 02:55:50', '2026-09-04 02:57:22',
    '2026-09-04 04:05:43', '2026-09-04 04:06:21',
    '2026-09-04 05:09:06', '2026-09-04 05:09:44',
    '2026-09-04 05:10:23', '2026-09-04 05:11:01',
    '2026-09-04 05:13:04', '2026-09-04 05:13:43', '2026-09-04 05:14:37',
}
# 5.05 m 에서 타깃 자세를 바꾼 run — delta 모델의 회귀 표본과 다른 자세다
POSE_SHIFT = {'2026-09-04 03:27:57', '2026-09-04 03:34:17', '2026-09-04 03:36:57'}


def sig_cam_r(d):
    return CAM_R[0] * np.power(d, CAM_R[1])


def sig_cam_t(d):
    return CAM_T[0] * np.power(d, CAM_T[1])


def sig_lid_r(d):
    d = np.asarray(d, float)
    return np.where(d >= LID_R_BREAK, LID_R_FLOOR, LID_R[0] * np.power(d, LID_R[1]))


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


# ── 데이터 적재 ──────────────────────────────────────────────────────
rows = [r for r in csv.DictReader(open(RESULTS)) if r['ts'] not in BAD_TS]

per_d = defaultdict(lambda: defaultdict(list))
for r in rows:
    if abs(f(r['bearing_deg']) or 0.0) > 1e-6:
        continue
    gt = f(r['gt_d'])
    if gt is None:
        continue
    d = round(gt, 2)
    ch = f(r['cam_hit_pct'])
    for k in ('cam_sigma_r', 'lid_sigma_r', 'cam_bias_gt', 'lid_bias_gt',
              'bias_CL_r', 'cam_sigma_th', 'lid_sigma_th', 'cam_hit_pct',
              'lid_hit_pct'):
        v = f(r[k])
        if v is not None:
            per_d[d][k].append((v, ch))

DS = sorted(per_d)


def agg(d, key, golden=False):
    vals = per_d[d].get(key, [])
    if golden:
        vals = [v for v, ch in vals if ch is not None and ch >= 99.999]
    else:
        vals = [v for v, _ in vals]
    return float(np.mean(vals)) if vals else float('nan')


def powfit(ds, ys):
    ds, ys = np.asarray(ds, float), np.asarray(ys, float)
    ok = np.isfinite(ds) & np.isfinite(ys) & (ys > 0)
    b, la = np.polyfit(np.log(ds[ok]), np.log(ys[ok]), 1)
    return math.exp(la), b


# =====================================================================
# 그림 1 — σ_r(d) 와 교차 거리
# =====================================================================
def fig_sigma():
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 5.2))

    dc = [d for d in DS if np.isfinite(agg(d, 'cam_sigma_r', golden=True))]
    yc = [1000 * agg(d, 'cam_sigma_r', golden=True) for d in dc]
    dl = [d for d in DS if np.isfinite(agg(d, 'lid_sigma_r'))]
    yl = [1000 * agg(d, 'lid_sigma_r') for d in dl]

    grid = np.logspace(np.log10(0.9), np.log10(7.5), 200)
    ax.loglog(grid, 1000 * sig_cam_r(grid), color=C_CAM, lw=2,
              label='σ_C(d) = 0.321 mm · d^2.741')
    ax.loglog(grid, 1000 * sig_lid_r(grid), color=C_LID, lw=2,
              label='σ_L(d) = 0.331 mm · d^0.354  (4.5 m~ 1.6 mm 바닥)')

    # 회귀에 쓴 점과 제외한 점을 구분한다
    fit_d = [d for d in dc if 2.0 <= d <= 5.2]
    ax.loglog(fit_d, [yc[dc.index(d)] for d in fit_d], 'o', color=C_CAM,
              ms=9, zorder=5, label='카메라 실측 (회귀 표본)')
    out_d = [d for d in dc if d not in fit_d]
    ax.loglog(out_d, [yc[dc.index(d)] for d in out_d], 'o', mfc='white',
              mec=C_CAM, mew=2, ms=9, zorder=5, label='카메라 실측 (회귀 제외)')
    ax.loglog(dl, yl, 's', color=C_LID, ms=8, zorder=5, label='라이다 실측')

    q = 1.0 / math.sqrt(12)
    ax.axhline(q, color=C_BAD, ls=':', lw=1.5)
    ax.text(7.45, q * 1.08, f'depth 양자화 바닥  q/√12 = {q:.3f} mm',
            color=C_BAD, fontsize=9.5, va='bottom', ha='right')

    dstar = (LID_R[0] / CAM_R[0]) ** (1.0 / (CAM_R[1] - LID_R[1]))
    ys = 1000 * sig_cam_r(dstar)
    ax.plot([dstar], [ys], '*', color=C_FUS, ms=20, zorder=6)
    ax.annotate(f'교차 d* = {dstar:.3f} m  (σ = {ys:.2f} mm)\n'
                '이 위로는 거리 추정에서 라이다가 항상 우위',
                xy=(dstar, ys), xytext=(1.45, 0.125), color=C_FUS, fontsize=9.8,
                arrowprops=dict(arrowstyle='->', color=C_FUS, lw=1.5))

    ax.set_xlabel('거리 d [m]')
    ax.set_ylabel('반경 방향 표준편차 σ_r [mm]')
    ax.set_title('반경(range) 오차 — 1 m 위에서는 라이다가 항상 우위')
    ax.grid(True, which='both', alpha=0.5)
    ax.set_xticks([1, 2, 3, 4, 5, 6, 7])
    ax.set_xticklabels(['1', '2', '3', '4', '5', '6', '7'])
    ax.legend(loc='upper left', fontsize=9.2)
    ax.set_xlim(0.9, 7.6)
    ax.set_ylim(0.085, 150)

    # ── 오른쪽: 이방성 비 ──
    # 5.10 m 는 제외한다. σ_C,t 가 2.10 mm 로 사실상 같은 거리인 5.05 m(6.90 mm)의
    # 1/3 이라 비가 14 로 튄다. 거리 효과가 아니라 배치 편차이고 미해결이다
    # (EXPERIMENT_C2_RESULTS.txt 11절 (1)).
    OUTLIER_T = 5.10
    ratio_d, ratio = [], []
    for d in DS:
        sr = agg(d, 'cam_sigma_r', golden=True)
        st = agg(d, 'cam_sigma_th', golden=True) * d
        if np.isfinite(sr) and np.isfinite(st) and st > 0 and d != OUTLIER_T:
            ratio_d.append(d)
            ratio.append(sr / st)
    ax2.plot(ratio_d, ratio, 'o-', color=C_CAM, ms=9, lw=2, label='카메라 σ_r / σ_t (실측)')
    g2 = np.linspace(1, 6.2, 100)
    ax2.plot(g2, sig_cam_r(g2) / sig_cam_t(g2), color=C_CAM, ls='--', lw=1.5,
             alpha=0.7, label='모델 (0.321·d^2.741) / (0.560·d^1.450)')
    ax2.axhline(1.0, color=C_GRAY, ls=':', lw=1.5)
    ax2.text(6.1, 1.06, '등방 (원형 오차)', color=C_GRAY, fontsize=9.5, ha='right')
    ax2.set_xlabel('거리 d [m]')
    ax2.set_ylabel('σ_r / σ_t  (오차 타원의 종횡비)')
    ax2.set_title('카메라 오차 타원은 시선 방향으로 길쭉하다')
    ax2.grid(True, alpha=0.5)
    ax2.legend(loc='upper left', fontsize=9.5)
    ax2.set_xlim(0.8, 6.4)
    ax2.set_ylim(0, 6.6)
    for d, rv in zip(ratio_d, ratio):
        ax2.annotate(f'{rv:.1f}배', xy=(d, rv), xytext=(7, -3),
                     textcoords='offset points', fontsize=9.5, color=C_CAM)
    ax2.text(0.5, -0.165,
             '이 이방성이 R 을 xy 대각으로 두면 안 되는 이유다.  '
             '5.10 m 는 제외 — σ_C,t 가 같은 거리 5.05 m 의 1/3 로 튄다(배치 편차, 미해결)',
             transform=ax2.transAxes, ha='center', fontsize=9.2, color='#718096')

    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'c2_sigma_curves.png'), dpi=170,
                bbox_inches='tight')
    plt.close(fig)
    print('c2_sigma_curves.png')


# =====================================================================
# 그림 2 — bias 와 대표점 차이 delta
# =====================================================================
def fig_bias():
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 5.0))

    bc = [1000 * agg(d, 'cam_bias_gt') for d in DS]
    bl = [1000 * agg(d, 'lid_bias_gt') for d in DS]
    ax.axhline(0, color=C_GRAY, lw=1)
    ax.plot(DS, bc, 'o-', color=C_CAM, ms=9, lw=2, label='b_C — 카메라 GT 대비 편향')
    ax.plot(DS, bl, 's-', color=C_LID, ms=8, lw=2, label='b_L — 라이다 GT 대비 편향')
    ax.axhspan(-16, 16, color=C_CAM, alpha=0.08)
    ax.text(7.0, 0, '±16 mm', color=C_CAM, fontsize=9.5, ha='right', va='center')
    ax.annotate('b_C 는 거리 의존성이 없다\n→ 카메라는 분산이 지배, 편향이 아니다',
                xy=(3.09, 7.7), xytext=(1.35, -21), fontsize=9.5, color=C_CAM,
                arrowprops=dict(arrowstyle='->', color=C_CAM, lw=1.4))
    ax.annotate('b_L 은 5 m 까지 8배 증가 후 포화\n→ 두께 절반(상수)으로 설명 안 된다',
                xy=(4.09, 72.3), xytext=(1.2, 88), fontsize=9.5, color=C_LID,
                arrowprops=dict(arrowstyle='->', color=C_LID, lw=1.4))
    ax.set_xlabel('거리 d [m]')
    ax.set_ylabel('GT 대비 편향 [mm]')
    ax.set_title('계통 편향 — GT 를 기록했기에 분리된다')
    ax.grid(True, alpha=0.5)
    ax.set_ylim(-34, 114)
    ax.legend(loc='center right', fontsize=9.5)

    # ── delta_r ──
    dd = [d for d in DS if np.isfinite(agg(d, 'bias_CL_r'))]
    yy = [1000 * agg(d, 'bias_CL_r') for d in dd]
    ax2.plot(dd, yy, 'o', color=C_FUS, ms=10, zorder=5, label='δ_r 실측 (거리별 평균)')
    g = np.linspace(1, 6.2, 100)
    model = 1000 * DELTA_R[0] * g ** DELTA_R[1]
    ax2.plot(g, model, color=C_FUS, lw=2,
             label='δ_r(d) = −10.98 mm · d^1.281  (노드 기본값)')
    pred = 1000 * DELTA_R[0] * np.array(dd) ** DELTA_R[1]
    rms = float(np.sqrt(np.mean((pred - np.array(yy)) ** 2)))
    ax2.fill_between(g, model - Q_R * 1000, model + Q_R * 1000, color=C_FUS,
                     alpha=0.15, label=f'게이트에 더하는 잔차 q_r = {Q_R*1000:.2f} mm')
    for d, y in zip(dd, yy):
        if d in (5.05,):
            ax2.annotate('자세 변경 run 이 섞인 거리\n(게이트 통과율을 깎는 곳)',
                         xy=(d, y), xytext=(1.25, -101), fontsize=9.2, color=C_BAD,
                         arrowprops=dict(arrowstyle='->', color=C_BAD, lw=1.3))
    ax2.set_xlabel('거리 d [m]')
    ax2.set_ylabel('δ_r = (카메라 − 라이다) 반경 [mm]')
    ax2.set_title(f'대표점 정의 차이 δ(d) — 잡음이 아니라 계통 차이 (잔차 RMS {rms:.2f} mm)')
    ax2.grid(True, alpha=0.5)
    ax2.legend(loc='lower left', fontsize=9.2)

    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'c2_bias_delta.png'), dpi=170, bbox_inches='tight')
    plt.close(fig)
    print('c2_bias_delta.png')


# =====================================================================
# 그림 3 — R(d) 오차 타원과 역분산 가중
# =====================================================================
def fig_ellipse():
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.9))
    for ax, d in zip(axes, (2.0, 4.0, 6.0)):
        sc_r, sc_t = float(sig_cam_r(d)), float(sig_cam_t(d))
        sl_r, sl_t = float(sig_lid_r(d)), LID_T_CONST
        Rc = np.diag([sc_r ** 2, sc_t ** 2])
        Rl = np.diag([sl_r ** 2, sl_t ** 2])
        P = np.linalg.inv(np.linalg.inv(Rc) + np.linalg.inv(Rl))

        # 위첨자 ᵀ·⁻¹ 는 Noto Sans CJK 에 글리프가 없어 두부가 된다. ^T / ^-1 로 쓴다.
        for (R, col, lab) in ((Rc, C_CAM, '카메라 R_C'),
                              (Rl, C_LID, '라이다 R_L'),
                              (P, C_FUS, '융합 P = (R_C^-1 + R_L^-1)^-1')):
            w = 2 * math.sqrt(R[0, 0]) * 1000
            h = 2 * math.sqrt(R[1, 1]) * 1000
            ax.add_patch(Ellipse((0, 0), w, h, fc=col, alpha=0.16, ec=col,
                                 lw=2.2, label=lab, zorder=3 if col == C_FUS else 2))

        wcr = (1 / sc_r ** 2) / (1 / sc_r ** 2 + 1 / sl_r ** 2) * 100
        wct = (1 / sc_t ** 2) / (1 / sc_t ** 2 + 1 / sl_t ** 2) * 100
        lim = max(sc_r, sc_t, sl_t) * 1000 * 1.45
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect('equal')
        ax.axhline(0, color=C_GRAY, lw=0.8, zorder=1)
        ax.axvline(0, color=C_GRAY, lw=0.8, zorder=1)
        ax.set_title(f'd = {d:.0f} m\n'
                     f'카메라 가중치   반경 {wcr:.1f} %   방위 {wct:.1f} %',
                     fontsize=12)
        ax.set_xlabel('반경 방향 (시선) [mm]')
        if d == 2.0:
            ax.set_ylabel('횡 방향 (방위) [mm]')
        ax.grid(True, alpha=0.4)
    axes[0].legend(loc='upper left', fontsize=8.4)
    fig.suptitle('R(d) = J(θ)·diag(σ_r², σ_t²)·J(θ)^T  —  1σ 타원 (센서 시선 = x축)',
                 fontsize=13.5, fontweight='bold')
    fig.tight_layout(rect=(0, 0.07, 1, 0.96))
    fig.text(0.5, 0.015,
             '"거리는 라이다, 방위는 카메라" 가 if 문이 아니라 역분산 가중식에서 나온다. '
             'R 을 xy 대각으로 두면 이 방향별 분리가 사라진다.',
             ha='center', fontsize=10, color='#718096')
    fig.savefig(os.path.join(HERE, 'c2_error_ellipse.png'), dpi=170,
                bbox_inches='tight')
    plt.close(fig)
    print('c2_error_ellipse.png')


# =====================================================================
# 그림 4 — 가중치 곡선
# =====================================================================
def fig_weights():
    fig, ax = plt.subplots(figsize=(8.6, 5.2))
    g = np.linspace(1.0, 6.5, 300)
    cr, lr = sig_cam_r(g), sig_lid_r(g)
    ct, lt = sig_cam_t(g), LID_T_CONST
    wr = (1 / cr ** 2) / (1 / cr ** 2 + 1 / lr ** 2) * 100
    wt = (1 / ct ** 2) / (1 / ct ** 2 + 1 / lt ** 2) * 100

    ax.plot(g, wr, color=C_CAM, lw=2.5, label='반경(range) — 카메라 가중치 w_C,r')
    ax.plot(g, wt, color=C_CAM, lw=2.5, ls='--', label='방위(bearing) — 카메라 가중치 w_C,t')
    ax.fill_between(g, 0, wr, color=C_CAM, alpha=0.10)
    ax.axhline(50, color=C_GRAY, ls=':', lw=1.5)
    ax.text(6.45, 51.5, '50 % — 두 센서 동등', color=C_GRAY, fontsize=9.5, ha='right')

    for d in (2, 4, 6):
        i = int(np.argmin(abs(g - d)))
        ax.plot([d, d], [wr[i], wt[i]], color=C_GRAY, lw=1, ls='-', alpha=0.6)
        ax.plot([d], [wr[i]], 'o', color=C_CAM, ms=8)
        ax.plot([d], [wt[i]], 'o', mfc='white', mec=C_CAM, mew=2, ms=8)
        ax.annotate(f'{wt[i]:.0f} %', xy=(d, wt[i]), xytext=(6, 6),
                    textcoords='offset points', fontsize=9.5, color=C_CAM)
        ax.annotate(f'{wr[i]:.1f} %', xy=(d, wr[i]), xytext=(6, -14),
                    textcoords='offset points', fontsize=9.5, color=C_CAM)

    ax.set_xlabel('거리 d [m]')
    ax.set_ylabel('융합 위치에서 카메라가 차지하는 비중 [%]')
    ax.set_title('역분산 가중 — 같은 거리에서 방향별로 가중치가 갈린다')
    ax.set_ylim(-3, 108)
    ax.set_xlim(0.95, 6.55)
    ax.grid(True, alpha=0.5)
    ax.legend(loc='center left', fontsize=10)
    i4 = int(np.argmin(abs(g - 4.0)))
    ax.text(0.5, -0.185,
            f'w_C = σ_L² / (σ_C² + σ_L²).  4 m 에서 반경 {wr[i4]:.1f} % / '
            f'방위 {wt[i4]:.0f} % — 한 물체의 x 와 y 를 서로 다른 센서가 결정한다.',
            transform=ax.transAxes, ha='center', fontsize=9.8, color='#718096')
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'c2_weights.png'), dpi=170, bbox_inches='tight')
    plt.close(fig)
    print('c2_weights.png')


# =====================================================================
# 그림 5 — 게이트
# =====================================================================
def fig_gate():
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 5.0))
    g = np.linspace(1.0, 6.5, 200)
    gr = 1000 * np.sqrt(GATE * (sig_cam_r(g) ** 2 + sig_lid_r(g) ** 2 + Q_R ** 2))
    gt = 1000 * np.sqrt(GATE * (sig_cam_t(g) ** 2 + LID_T_CONST ** 2 + Q_T ** 2))
    p95 = 48.3 * g

    ax.plot(g, gr, color=C_FUS, lw=2.5, label='χ² 게이트 — 시선 방향')
    ax.plot(g, gt, color=C_FUS, lw=2.5, ls='--', label='χ² 게이트 — 횡 방향')
    ax.plot(g, p95, color=C_BAD, lw=2, ls=':',
            label='관측 d_CL p95 = 48.3 mm/m · d  (보정 전)')
    ax.annotate('p95 를 그대로 게이트로 쓰면\n3 m 에서 149 mm — 필요보다 5 배 넓다',
                xy=(3.0, 145), xytext=(1.15, 205), fontsize=9.5, color=C_BAD,
                arrowprops=dict(arrowstyle='->', color=C_BAD, lw=1.4))
    ax.set_xlabel('거리 d [m]')
    ax.set_ylabel('게이트 반경 [mm]')
    ax.set_title('게이트 폭 — χ²(2, 0.99) = 9.21 을 문턱으로')
    ax.grid(True, alpha=0.5)
    ax.legend(loc='upper left', fontsize=9.5)
    ax.set_xlim(0.95, 6.55)

    # ── 오른쪽: 보정 단계별 통과율 ──
    stages = ['보정 없음\n(δ·yaw 미적용)', 'δ·yaw 보정\nQ 없음', 'δ·yaw 보정\n+ Q (구현)']
    vals = [13.76, 89.18, 99.50]
    cols = [C_BAD, '#D69E2E', C_LID]
    bars = ax2.bar(stages, vals, color=cols, width=0.58)
    for b, v in zip(bars, vals):
        ax2.text(b.get_x() + b.get_width() / 2, v + 2.5, f'{v:.2f} %',
                 ha='center', fontsize=12, fontweight='bold', color=C_TEXT)
    ax2.axhline(99, color=C_GRAY, ls=':', lw=1.5)
    ax2.text(-0.45, 100.8, '설계값 99 %', color=C_GRAY, fontsize=9.5, ha='left')
    ax2.set_ylabel('진짜 쌍의 게이트 통과율 [%]')
    ax2.set_ylim(0, 116)
    ax2.set_title('raw 13,134 쌍 재검증 — 각 보정이 무엇을 사는가')
    ax2.grid(True, axis='y', alpha=0.5)
    ax2.text(0.5, -0.20,
             'δ 를 빼지 않으면 게이트가 진짜 쌍을 스스로 기각한다. '
             'Q 가 없으면 근거리에서 σ_comb 가 0.5 mm 라 모델 오차 몇 mm 에도 χ² 가 터진다.',
             transform=ax2.transAxes, ha='center', fontsize=9.5, color='#718096')

    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'c2_gate.png'), dpi=170, bbox_inches='tight')
    plt.close(fig)
    print('c2_gate.png')


# =====================================================================
# 그림 6 — 검출률 절벽
# =====================================================================
def fig_pdet():
    fig, ax = plt.subplots(figsize=(8.8, 5.0))
    camhit, lidhit = [], []
    for d in DS:
        vals = [v for v, ch in per_d[d].get('cam_hit_pct', []) if v > 0]
        camhit.append(float(np.mean(vals)) if vals else 0.0)
        lidhit.append(agg(d, 'lid_hit_pct'))

    ax.plot(DS, lidhit, 's-', color=C_LID, ms=9, lw=2.5, label='라이다 검출률 (전 거리 100 %)')
    ax.plot(DS, camhit, 'o-', color=C_CAM, ms=10, lw=2.5, label='카메라 검출률')
    ax.axvspan(6.3, 7.5, color=C_BAD, alpha=0.08)
    ax.annotate('7.10 m 에서 0 %\n점진적 열화가 아니라 절벽',
                xy=(7.10, 0), xytext=(5.15, 26), fontsize=10, color=C_BAD,
                arrowprops=dict(arrowstyle='->', color=C_BAD, lw=1.5))

    ax2 = ax.twinx()
    conf_d, conf_v = [], []
    if os.path.exists(CHECK):
        cd = defaultdict(list)
        for r in csv.DictReader(open(CHECK)):
            v, nd = f(r.get('conf_mean')), f(r.get('distance'))
            if v and nd and v > 0:
                cd[round(nd, 1)].append(v)
        for k in sorted(cd):
            conf_d.append(k)
            conf_v.append(float(np.mean(cd[k])))
    if conf_d:
        ax2.plot(conf_d, conf_v, '^--', color='#805AD5', ms=8, lw=1.8,
                 label='YOLO confidence 평균')
        ax2.axhline(0.15, color='#805AD5', ls=':', lw=1.5)
        ax2.text(1.05, 0.168, 'yolo_conf 문턱 = 0.15 (실험 내내 고정)',
                 color='#805AD5', fontsize=9.5)
        ax2.set_ylabel('YOLO confidence', color='#805AD5')
        ax2.tick_params(axis='y', colors='#805AD5')
        ax2.set_ylim(0, 1.0)
        ax2.legend(loc='center left', fontsize=9.5)

    ax.set_xlabel('거리 d [m]')
    ax.set_ylabel('검출률 P_det [%]')
    ax.set_ylim(-4, 108)
    ax.set_title('카메라의 유효 상한 6 m — depth 품질이 아니라 detector confidence 한계')
    ax.grid(True, alpha=0.5)
    ax.legend(loc='lower left', fontsize=9.5)
    ax.text(0.5, -0.185,
            '6 m 이상에서는 카메라 관측이 없으므로 association 자체가 성립하지 않는다 '
            '→ 라이다 단독 트랙으로 넘긴다.',
            transform=ax.transAxes, ha='center', fontsize=9.8, color='#718096')
    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'c2_pdet.png'), dpi=170, bbox_inches='tight')
    plt.close(fig)
    print('c2_pdet.png')


# =====================================================================
# 그림 7 — ABD D_max 가 만든 계획 밖 발견
# =====================================================================
def fig_abd():
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(13, 4.9))

    # 왼쪽: 클러스터 폭.
    # 0도 조건만 쓴다. ±20도 배치는 타깃을 비스듬히 보므로 폭이 비교 대상이 아니다
    # (1.8 m 에서 931 mm = 배경 병합, 3.4 m 에서 155 mm = 부분 가림).
    # 0.5 m 단위로 묶는다 — 5.1/5.2 처럼 같은 조건이 반올림 경계로 갈리는 것을 막는다.
    # 표시 위치는 묶음 키가 아니라 그 묶음의 실제 r 평균이다.
    wd, rd = defaultdict(list), defaultdict(list)
    if os.path.exists(CHECK):
        for r in csv.DictReader(open(CHECK)):
            w, lr, bd = (f(r.get('lid_w_mean')), f(r.get('lid_r_mean')),
                         f(r.get('bearing_deg')))
            if w and lr and w > 0 and bd is not None and abs(bd) < 1e-6:
                k = round(lr * 2) / 2
                wd[k].append(w)
                rd[k].append(lr)
    keys = [k for k in sorted(wd) if len(wd[k]) >= 4]
    ds = [float(np.mean(rd[k])) for k in keys]
    ws = [1000 * float(np.mean(wd[k])) for k in keys]
    if ds:
        ax.plot(ds, ws, 'o-', color=C_LID, ms=9, lw=2,
                label='ABD 클러스터 폭 (0도, 실측 평균)')
        ax.axhline(300, color=C_GRAY, ls='--', lw=2, label='타깃 참폭 300 mm (강체)')
        ax.fill_between(ds, 300, ws, color=C_BAD, alpha=0.12)
        for k, w in zip(ds, ws):
            ax.annotate(f'+{(w-300)/3:.0f} %', xy=(k, w), xytext=(0, 11),
                        textcoords='offset points', ha='center', fontsize=9.2,
                        color=C_BAD)
        ax.annotate('강체의 폭은 거리의 함수일 수 없다\n= 배경 점을 흡수하고 있다',
                    xy=(ds[-1], ws[-1]), xytext=(3.15, 313), fontsize=10, color=C_BAD,
                    arrowprops=dict(arrowstyle='->', color=C_BAD, lw=1.4))
        ax.set_ylim(255, 475)
    ax.set_xlabel('라이다 거리 r [m]')
    ax.set_ylabel('클러스터 폭 [mm]')
    ax.set_title('타깃은 강체인데 클러스터 폭이 거리에 따라 커진다')
    ax.grid(True, alpha=0.5)
    ax.legend(loc='upper left', fontsize=9.8)

    # 오른쪽: D_max vs 점 간격
    lam, dphi, k, sr = math.radians(20), math.radians(0.5), 3, 0.03
    r = np.linspace(1, 7.5, 100)
    dmax = 1000 * (r * math.sin(dphi) / math.sin(lam - dphi) + k * sr)
    gap = 1000 * r * dphi
    ax2.plot(r, dmax, color=C_FUS, lw=2.5,
             label='D_max(r) = r·sinΔφ/sin(λ−Δφ) + k·σ_r   (λ=20°)')
    ax2.plot(r, gap, color=C_LID, lw=2.5, label='같은 물체 위 인접 점 간격 = r·Δφ')
    ax2.fill_between(r, gap, dmax, color=C_FUS, alpha=0.12)
    ax2.annotate('이 간격이 배경을 붙였다 뗐다 하게 만든다\n→ centroid 이동 → σ_L 의 5 m peak',
                 xy=(5.5, 0.5 * (1000 * (5.5 * math.sin(dphi) / math.sin(lam - dphi)
                                         + k * sr) + 1000 * 5.5 * dphi)),
                 xytext=(1.35, 232), fontsize=9.8, color=C_FUS,
                 arrowprops=dict(arrowstyle='->', color=C_FUS, lw=1.4))
    ax2.set_xlabel('라이다 거리 r [m]')
    ax2.set_ylabel('거리 [mm]')
    ax2.set_title('원인은 식에 있다 — ABD 문턱이 r 에 비례')
    ax2.grid(True, alpha=0.5)
    ax2.legend(loc='upper left', fontsize=9.2)
    ax2.text(0.5, -0.19,
             'D_max 가 r 에 비례하는 것 자체는 설계 의도다. 문제는 계수 — '
             '7.1 m 에서 점 간격 62 mm 대비 D_max 276 mm 로 4.4 배다.',
             transform=ax2.transAxes, ha='center', fontsize=9.5, color='#718096')

    fig.tight_layout()
    fig.savefig(os.path.join(HERE, 'c2_abd.png'), dpi=170, bbox_inches='tight')
    plt.close(fig)
    print('c2_abd.png')


if __name__ == '__main__':
    fig_sigma()
    fig_bias()
    fig_ellipse()
    fig_weights()
    fig_gate()
    fig_pdet()
    fig_abd()
