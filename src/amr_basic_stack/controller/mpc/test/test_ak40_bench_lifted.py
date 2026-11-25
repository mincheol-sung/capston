"""tools/ak40_bench.py lifted — 바퀴를 띄운 토크 단계 시험 (바퀴 회전자 식별 지시서 W1, 2026-11-24).

CAN·실제 시계 없이 가짜 모터 (J·ω̇ = τ − τc·sgn ω − b·ω, 정지 마찰) 로 단계표·바퀴 기준 부호·안전 정지를 확인한다.
"""
import argparse
import importlib.util
import math
import os

import pytest

from amr_mpc.ak40 import protocol as P

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("ak40_bench", os.path.join(HERE, "..", "tools", "ak40_bench.py"))
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)

LIM = P.MitLimits()
# 2026-11-24 사용자 승인 순서 (바퀴 기준): 출발용 0.08 을 0.5 s, 네 값을 4 s 씩, 마지막에 0 (자유 감속)
TAUS = [0.08, 0.045, 0.030, 0.040, 0.035, 0.045, 0.0]
HOLDS = [0.5, 4.0, 4.0, 4.0, 4.0, 4.0, 1.5]
TC, BV, JW = 0.0174, 0.001066, 8.9e-4            # torque_plant.yaml FL 값


class FakeMotor:
    """모터 하나 (모터 기준 부호). send() 는 받은 순간의 상태로 답하고, sleep() 이 시간을 흘린다."""

    def __init__(self, static=0.05, drop_after=None, error_after=None):
        self.t, self.v, self.p, self.tau = 0.0, 0.0, 0.0, 0.0
        self.static, self.drop_after, self.error_after = static, drop_after, error_after
        self.cmds = []                               # (t, 받은 모터 토크)

    def clock(self):
        return self.t

    def sleep(self, dt):
        n = max(1, int(round(dt / 1e-4)))
        h = dt / n
        for _ in range(n):
            if abs(self.v) < 1e-9 and abs(self.tau) <= self.static:
                self.v = 0.0                         # 정지 마찰에 붙어 있다
            else:
                s = math.copysign(1.0, self.v if abs(self.v) > 1e-9 else self.tau)
                v_new = self.v + h * (self.tau - TC * s - BV * self.v) / JW
                if self.v != 0.0 and v_new * self.v < 0.0 and abs(self.tau) <= self.static:
                    v_new = 0.0                      # 서면 다시 붙는다
                self.v = v_new
            self.p += h * self.v
        self.t += dt

    def send(self, data):
        if self.drop_after is not None and self.t >= self.drop_after:
            return None
        _, _, kp, kd, t = P.unpack_command(LIM, data)
        assert kp == 0.0 and kd == 0.0               # 토크만 낸다
        self.tau = t
        self.cmds.append((self.t, t))
        err = 1 if (self.error_after is not None and self.t >= self.error_after) else 0
        return P.unpack_reply(LIM, P.pack_reply(LIM, 1, P.wrap_position(self.p, LIM), self.v, self.tau, temp_c=37.0, error=err))


def run(sign, taus=TAUS, holds=HOLDS, v_max=38.7, motor=None, **kw):
    m = motor or FakeMotor()
    plan = bench.lifted_plan(taus, holds, sign, LIM, 0.1)
    rows, info = bench.run_lifted(m, plan, sign, LIM, 0.01, v_max, clock=m.clock, sleep=m.sleep, **kw)
    return m, plan, rows, info


def test_plan_sign_and_actual_torque():
    """바퀴 기준 + 를 왼쪽 바퀴 (장착 부호 −1) 는 모터 − 로 보낸다. 실제 나가는 값은 2.44 mN·m 격자 — "0" 은 모터 +1.22 mN·m."""
    left = bench.lifted_plan([0.08, 0.045, 0.0], [0.5, 4, 1.5], -1, LIM, 0.1)
    right = bench.lifted_plan([0.08, 0.045, 0.0], [0.5, 4, 1.5], 1, LIM, 0.1)
    assert [s["tau_motor"] for s in left] == [-0.08, -0.045, -0.0]
    assert left[0]["tau_q"] == pytest.approx(0.07937, abs=1e-5) and right[0]["tau_q"] == pytest.approx(0.07937, abs=1e-5)
    assert left[1]["tau_q"] == pytest.approx(0.04518, abs=1e-5)
    assert right[2]["tau_q"] == pytest.approx(0.001221, abs=1e-6)      # 모터 +1.22 mN·m = 오른쪽 바퀴 전진 쪽
    assert left[2]["tau_q"] == pytest.approx(-0.001221, abs=1e-6)      # 왼쪽 바퀴에는 후진 쪽
    assert bench.quantize_torque(LIM, 0.03) == pytest.approx(0.03053, abs=1e-5)


@pytest.mark.parametrize("taus, holds, cap", [
    ([0.08, 0.2], [0.5, 4], 0.1),            # 한도 초과 — 자르지 않고 거부
    ([0.08, 0.045], [0.5], 0.1),             # 개수 다름
    ([0.08], [11.0], 0.1),                   # 단계 10 s 초과
    ([0.04] * 8, [9.0] * 8, 0.1),            # 합 60 s 초과
    ([float("nan")], [1.0], 0.1),
    ([0.05], [1.0], 0.0),                    # 한도 0
])
def test_plan_refuses(taus, holds, cap):
    with pytest.raises(ValueError):
        bench.lifted_plan(taus, holds, 1, LIM, cap)


@pytest.mark.parametrize("sign", [-1.0, 1.0])
@pytest.mark.parametrize("direction", [1.0, -1.0])            # 바퀴 기준 전진·후진
def test_sequence_runs_in_wheel_frame(sign, direction):
    """승인 순서를 끝까지 낸다. 기록은 바퀴 기준 (전진 +), 일정 속도는 (실제 나간 τ − τc)/b, 마지막은 자유 감속."""
    m, plan, rows, info = run(sign, taus=[direction * t for t in TAUS])
    assert info["stop"] == "완료" and info["missed"] == 0
    assert all(len(r) == len(bench.LIFTED_COLUMNS) for r in rows)
    assert 0 <= len(rows) - sum(HOLDS) / 0.01 <= len(HOLDS)                     # 단계마다 끝에서 한 주기까지 더 나갈 수 있다
    # 모터가 받은 부호 = sign × 바퀴 기준 부호
    kick = [t for tt, t in m.cmds if tt < 0.4]
    assert all(math.copysign(1.0, t) == sign * direction for t in kick)
    summ = bench.lifted_step_summary(rows, plan)
    for st, s in zip(plan[1:6], summ[1:6]):
        assert s["w_end"] * direction > 0.0                                   # 바퀴 기준 속도 부호 = 명령 부호
        assert abs(s["w_end"]) == pytest.approx((abs(st["tau_q"]) - TC) / BV, rel=0.02)
        assert s["tau_reply_end"] == pytest.approx(st["tau_q"], abs=1.3e-3)
    assert 20.0 < info["v_peak"] < 38.7                                       # 출발용 0.5 s 로는 속도 상한에 안 닿는다
    coast = [r for r in rows if r[9] == 6]
    assert coast[0][7] == 0.0 and coast[0][8] == pytest.approx(sign * 0.001221, abs=1e-6)
    assert abs(coast[0][3]) > 20.0 and abs(coast[-1][3]) < 0.5               # 약 26 rad/s 에서 시작해 선다
    assert info["temp_start"] == 37.0 and info["temp_end"] == 37.0


def test_speed_limit_trips_to_zero_torque():
    m, plan, rows, info = run(-1.0, v_max=15.0)
    assert info["stop"].startswith("속도 상한")
    t_trip = next(r[0] for r in rows if r[1] == "trip")
    assert t_trip < 0.5                                                       # 출발용 단계 안에서 걸린다
    after = [t for tt, t in m.cmds if tt > t_trip + 0.011]
    assert after and max(abs(t) for t in after) < 1.3e-3                      # 그 뒤로는 토크 0 ("0" = 1.22 mN·m) 만
    trip = [r for r in rows if r[1] == "trip"]
    assert 0.95 <= trip[-1][0] - trip[0][0] <= 1.05                           # 1 s 더 기록하고 끝
    assert abs(trip[-1][3]) < abs(trip[0][3])                                 # 감속 중
    assert max(r[9] for r in rows) == 0                                       # 다음 단계로 안 넘어갔다


def test_start_failure_stops_after_first_step():
    m, plan, rows, info = run(1.0, motor=FakeMotor(static=0.2))               # 0.08 로는 정지 마찰을 못 넘는다
    assert info["stop"].startswith("출발 실패")
    assert max(tt for tt, t in m.cmds if abs(t) > 0.01) < 0.5 + 1e-6          # 첫 단계 (0.5 s) 뒤로 토크를 안 준다
    assert info["v_peak"] < 0.05                                              # 안 돌았다 (속도 0 은 12 비트 격자에서 0.011 로 읽힌다)


def test_missing_replies_stop():
    m, plan, rows, info = run(1.0, motor=FakeMotor(drop_after=1.0), max_miss=10)
    assert info["stop"].startswith("응답 없음")
    assert rows[-1][0] < 1.0 and info["missed"] >= 10


def test_motor_error_stops():
    m, plan, rows, info = run(1.0, motor=FakeMotor(error_after=1.0))
    assert info["stop"].startswith("모터 에러 1")
    assert max(tt for tt, t in m.cmds if abs(t) > 0.01) < 1.0 + 0.011


def _args(**kw):
    d = dict(id=1, taus="0.08,0.045,0", holds="0.5,4,1.5", hold=3.0, frame="wheel", v_max=0.0, tau_cap=0.1)
    d.update(kw)
    return argparse.Namespace(**d)


def test_prepare_reads_direction_and_rated_speed():
    """config/ak40.yaml: ID 1·3 (FL·RL) 은 장착 부호 −1, ID 2·4 는 +1. 속도 상한 기본 = 정격 370 rpm."""
    p1, p2 = bench.lifted_prepare(_args(id=1)), bench.lifted_prepare(_args(id=2))
    assert (p1["wheel"], p1["direction"], p1["sign"]) == (0, -1, -1.0)
    assert (p2["wheel"], p2["direction"], p2["sign"]) == (1, 1, 1.0)
    assert p1["v_max"] == pytest.approx(370.0 * 2.0 * math.pi / 60.0)
    assert bench.lifted_prepare(_args(id=1, frame="motor"))["sign"] == 1.0    # 모터 기준이면 부호를 안 바꾼다
    assert [s["hold"] for s in bench.lifted_prepare(_args(holds=""))["plan"]] == [3.0, 3.0, 3.0]   # --holds 없으면 --hold


@pytest.mark.parametrize("kw", [dict(id=9), dict(taus="0.08,0.3,0"), dict(v_max=50.0), dict(holds="0.5,4")])
def test_prepare_refuses_before_opening_can(kw):
    with pytest.raises(SystemExit):
        bench.lifted_prepare(_args(**kw))


def test_lifted_main_writes_csv_and_meta(tmp_path):
    """파일 이름 YYYYMMDD_HHMM_lifted_<바퀴>_<tag>.csv + .json. CSV 는 기존 분석 도구 (jw_step_analysis.read_csv) 로도 읽힌다."""
    import csv, glob, json, importlib.util as iu
    a = _args(id=1, taus=",".join(str(t) for t in TAUS), holds=",".join(str(h) for h in HOLDS),
              csv="", out_dir=str(tmp_path), tag="fwd_r1", start_min=3.0, max_miss=10)
    prep = bench.lifted_prepare(a)
    m = FakeMotor()
    m.lim, m.period = LIM, 0.01
    info = bench.lifted_main(m, a, prep, clock=m.clock, sleep=m.sleep)
    assert info["stop"] == "완료"
    files = sorted(glob.glob(str(tmp_path / "*_lifted_FL_fwd_r1.csv")))
    assert len(files) == 1
    rows = list(csv.DictReader(open(files[0])))
    assert list(rows[0].keys()) == bench.LIFTED_COLUMNS
    assert float(rows[100]["v"]) > 0.0                                        # FL 전진 = 바퀴 기준 +
    meta = json.load(open(files[0][:-4] + ".json"))
    assert meta["wheel"] == "FL" and meta["direction"] == -1 and meta["frame"] == "wheel" and meta["n_rows"] == len(rows)
    assert [round(s["tau"], 3) for s in meta["plan"]] == TAUS
    assert meta["plan"][1]["tau_q"] == pytest.approx(0.04518, abs=1e-5)
    assert meta["config"]["J_w"] == pytest.approx(8.9e-4) and len(meta["summary"]) == len(TAUS)
    # 같은 분에 한 판 더 — 덮어쓰지 않는다
    m2 = FakeMotor()
    m2.lim, m2.period = LIM, 0.01
    bench.lifted_main(m2, a, prep, clock=m2.clock, sleep=m2.sleep)
    assert len(glob.glob(str(tmp_path / "*_lifted_FL_fwd_r1*.csv"))) == 2
    # 기존 분석 도구로 읽기 (열 이름으로 읽으므로 뒤에 붙은 열은 무시된다)
    sp = iu.spec_from_file_location("jw_step_analysis", os.path.join(HERE, "..", "tools", "jw_step_analysis.py"))
    jwa = iu.module_from_spec(sp)
    sp.loader.exec_module(jwa)
    t, mode, v, tr, tc = jwa.read_csv(files[0])
    assert len(t) == len(rows) and set(mode) == {"torque"} and tc[0] == pytest.approx(0.08)
