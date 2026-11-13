#!/usr/bin/env python3
# WP3 — 스텝 로그에서 FOPDT 파라미터 추정
# ---------------------------------------------------------------
#         K · e^(-theta*s)                     v(k+1) = a*v(k) + b*u(k-d)
# G(s) = -----------------      ZOH 이산화 ->
#            tau*s + 1                         tau = -Ts/ln a,  K = b/(1-a),  theta = d*Ts
#
# ARX 최소자승으로 (a, b) 를 풀고 d 는 0..DMAX 를 훑어 잔차 최소인 값을 고른다.
# 스텝의 상승 에지 구간만 회귀에 넣는다 (정지 구간은 정보가 없다).
#
# 사용:  python3 fit_fopdt.py <csv> [<csv> ...]
#   wp2_log_*.csv (3축) 와 velocity_step_test_*.csv (vx 전용) 둘 다 읽는다.
# ---------------------------------------------------------------
import csv, sys
import numpy as np

DMAX  = 8            # 데드타임 스윕 상한 [스텝]
PRE   = 25           # 에지 앞으로 포함할 샘플 수
POST  = 100          # 에지 뒤로 포함할 샘플 수
SPREAD_GATE = 0.05   # 바퀴 간 a 편차가 이보다 크면 축레벨 가정 경고
AXIS = {0: "vx", 1: "vy", 2: "wz"}

# base.yaml. 몸체 twist 명령을 바퀴 명령으로 바꿀 때만 쓴다.
# wp2_log 는 명령이 몸체 twist[m/s, rad/s] 인데 측정은 바퀴속도[rad/s] 라,
# 그대로 회귀하면 K 가 기구학 변환(1/r=25, k/r=5)을 주워담는다. 축이 아니라 단위 문제다.
R_WHEEL = 0.04
K_GEOM  = (0.1897 + 0.207) / 2.0        # (wheel_base + robot_width)/2 = 0.19835

# 메카넘 역기구학. 부호는 roboclaw_mecanum_node.twist_to_wheels 와 같다.
WHEEL_SIGNS = ((1, -1, -1), (1, 1, 1), (1, 1, -1), (1, -1, 1))   # FL, FR, RL, RR

# 두 CSV 형식의 열 이름 매핑: (시각, 축, 명령, 바퀴4)
LAYOUTS = [
    ("wp2_log", "t_s", "active", ("cvx", "cvy", "cw"),
     ("w0", "w1", "w2", "w3")),
    ("velocity_step_test", "time_s", None, ("cmd_fl_rad_s",),
     ("meas_fl_rad_s", "meas_fr_rad_s", "meas_rl_rad_s", "meas_rr_rad_s")),
]


def load(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f"{path}: 빈 파일")
    for name, tcol, acol, ucols, wcols in LAYOUTS:
        if tcol in rows[0] and wcols[0] in rows[0]:
            break
    else:
        raise SystemExit(f"{path}: 알 수 없는 열 구성")

    def num(c):
        out = np.empty(len(rows))
        for i, r in enumerate(rows):
            try: out[i] = float(r[c])
            except (TypeError, ValueError): out[i] = np.nan
        return out

    t = num(tcol)
    W = np.nan_to_num(np.column_stack([num(c) for c in wcols]))
    if acol:
        # 3축 로그: 명령이 몸체 twist 다. 역기구학으로 바퀴 명령[rad/s] 으로 바꿔야
        # 측정(바퀴속도)과 단위가 맞고 K 가 추종 이득이 된다.
        ax = np.nan_to_num(num(acol)).astype(int)
        vx, vy, wz = (np.nan_to_num(num(c)) for c in ucols)
        U = np.column_stack([(sx * vx + sy * vy + sw * K_GEOM * wz) / R_WHEEL
                             for sx, sy, sw in WHEEL_SIGNS])
    else:
        # vx 전용 로그: 이미 바퀴 명령[rad/s] 이고 네 바퀴가 같다.
        ax = np.zeros(len(t), int)
        U = np.tile(np.nan_to_num(num(ucols[0]))[:, None], (1, 4))
    pwm = np.column_stack([num(f"pwm{i}") for i in range(4)]) if "pwm0" in rows[0] else None
    # 진폭: 로거가 amp 열을 쓰면 그것, 아니면 명령 절댓값에서 (구형 로그 호환)
    if "amp" in rows[0]:
        amp = np.nan_to_num(num("amp"))
    elif acol:
        amp = np.max(np.abs(np.column_stack([np.nan_to_num(num(c)) for c in ucols])), axis=1)
    else:
        amp = np.abs(np.nan_to_num(num(ucols[0])))
    return name, t, ax, U, W, pwm, np.round(amp, 3)


def edges(u, mask):
    """정지->구동 전환 지점 주변 구간. (시작, 끝) 인덱스 목록."""
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return []
    out, prev = [], 0.0
    for i in idx:
        if prev == 0.0 and u[i] != 0.0:
            out.append((max(idx[0], i - PRE), min(idx[-1] + 1, i + POST)))
        prev = u[i]
    return out


def fit(y, u, segs, Ts):
    """ARX 최소자승 + d 스윕. 잔차 최소인 (a, b, d) 를 돌려준다."""
    best = None
    for d in range(DMAX + 1):
        P, Y = [], []
        for a, b in segs:
            for k in range(a + 1, b):
                if k - 1 - d < 0:
                    continue
                P.append((y[k - 1], u[k - 1 - d])); Y.append(y[k])
        if len(P) < 20:
            continue
        P, Y = np.array(P), np.array(Y)
        th, *_ = np.linalg.lstsq(P, Y, rcond=None)
        rms = float(np.sqrt(np.mean((Y - P @ th) ** 2)))
        if best is None or rms < best[0]:
            best = (rms, d, th)
    if best is None:
        return None
    rms, d, (a, b) = best
    return dict(a=float(a), b=float(b), d=d, rms=rms,
                tau=-Ts / np.log(a) if 0 < a < 1 else np.nan,
                K=b / (1 - a) if a < 1 else np.nan)


AMP_GATE = 0.10      # 진폭 간 tau 편차가 이보다 크면 속도 의존 구동계 — 단일 1차 모델 재검토


def report(path):
    name, t, ax, U, W, pwm, amp = load(path)
    Ts = float(np.median(np.diff(t)))
    print(f"\n{'='*62}\n {path}")
    print(f" 형식 {name} · {len(t)}행 · Ts={Ts*1000:.1f}ms · {t[-1]-t[0]:.1f}s")
    if pwm is not None:
        pk = np.nanmax(np.abs(pwm))
        print(f" PWM 피크 {pk:.0f}%  {'← 포화. 해당 구간은 피팅에서 빼야 한다' if pk > 90 else '(포화 없음)'}")
    print("=" * 62)

    for a_id in sorted(set(ax.tolist())):
        mask = ax == a_id
        segs = edges(U[:, 0], mask)
        if not segs:
            continue
        print(f"\n[{AXIS.get(a_id, a_id)}]  스텝 에지 {len(segs)}개")
        print(f"{'바퀴':>5}{'a':>9}{'tau[ms]':>10}{'K':>9}{'d':>4}{'theta[ms]':>11}{'잔차rms':>10}")
        A, T_, K_ = [], [], []
        for i, wn in enumerate(("FL", "FR", "RL", "RR")):
            f = fit(W[:, i], U[:, i], segs, Ts)
            if f is None:
                print(f"{wn:>5}   데이터 부족"); continue
            A.append(f["a"]); T_.append(f["tau"]); K_.append(f["K"])
            print(f"{wn:>5}{f['a']:>9.4f}{f['tau']*1000:>10.1f}{f['K']:>9.4f}"
                  f"{f['d']:>4}{f['d']*Ts*1000:>11.0f}{f['rms']:>10.4f}")
        if len(A) < 2:
            continue
        # 게이트는 a 편차로 본다. 축간 누설은 극점 차이에서 나오기 때문이다.
        # tau 편차도 같이 찍는다 — tau = -Ts/ln a 라 a≈1 근처에서 크게 증폭되어,
        # 둘을 섞어 보면 통과/탈락 판정이 뒤집힌다 (같은 데이터에서 a 0.9% vs tau 4.5%).
        sa = (max(A) - min(A)) / abs(np.mean(A))
        st = (max(T_) - min(T_)) / abs(np.mean(T_))
        print(f"      tau 평균 {np.mean(T_)*1000:.1f} ms   K 평균 {np.mean(K_):.4f}")
        print(f"      바퀴 간 편차:  a {sa*100:.1f}%  (게이트 {SPREAD_GATE*100:.0f}%)"
              f"   tau {st*100:.1f}% (참고)")
        if sa > SPREAD_GATE:
            print(f"      ⚠ a 편차가 게이트 초과 — 축레벨(축별 단일 모델) 가정이 흔들린다")

        # ── 진폭별: 같은 에지를 진폭으로 갈라 따로 맞춘다. tau 가 진폭에 따라 변하면 속도 의존.
        by_amp = {}
        for s0, s1 in segs:
            a_val = float(amp[s0:s1].max())
            by_amp.setdefault(a_val, []).append((s0, s1))
        if len(by_amp) >= 2:
            print(f"      진폭별 (바퀴 4개 평균)  {'진폭':>7}{'에지':>5}{'tau[ms]':>10}{'K':>9}{'theta[ms]':>11}{'잔차rms':>10}")
            taus = {}
            for a_val in sorted(by_amp):
                fs = [fit(W[:, i], U[:, i], by_amp[a_val], Ts) for i in range(4)]
                fs = [f for f in fs if f is not None]
                if not fs:
                    continue
                tau_m = np.mean([f["tau"] for f in fs]); taus[a_val] = tau_m
                print(f"      {'':24s}{a_val:>7.2f}{len(by_amp[a_val]):>5}{tau_m*1000:>10.1f}"
                      f"{np.mean([f['K'] for f in fs]):>9.4f}{np.mean([f['d'] for f in fs])*Ts*1000:>11.0f}"
                      f"{np.mean([f['rms'] for f in fs]):>10.4f}")
            if len(taus) >= 2:
                tv = np.array(list(taus.values())); spread = (tv.max() - tv.min()) / tv.mean()
                verdict = ("속도 의존 — 진폭별 모델 또는 게인 스케줄 검토" if spread > AMP_GATE
                           else "단일 1차 모델 유지 가능")
                print(f"      진폭 간 tau 편차 {spread*100:.1f}%  (게이트 {AMP_GATE*100:.0f}%)  → {verdict}")


def demo():
    """알려진 (a,b,d) 로 만든 신호를 되맞춰 피팅 자체를 검산한다."""
    Ts, a, b, d = 0.02, 0.8179, 0.1821, 1
    n = 600
    u = np.zeros(n)
    for s in range(0, n, 150):
        u[s + 25:s + 125] = 7.5
    y = np.zeros(n)
    for k in range(1, n):
        y[k] = a * y[k - 1] + b * (u[k - 1 - d] if k - 1 - d >= 0 else 0.0)
    f = fit(y, u, edges(u, np.ones(n, bool)), Ts)
    assert abs(f["a"] - a) < 1e-6 and f["d"] == d, f
    assert abs(f["K"] - b / (1 - a)) < 1e-6, f
    print(f"self-check OK: a={f['a']:.4f} d={f['d']} tau={f['tau']*1000:.1f}ms K={f['K']:.4f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        demo(); raise SystemExit("사용: python3 fit_fopdt.py <csv> [<csv> ...]")
    for p in sys.argv[1:]:
        report(p)
