"""AK40-10 CAN 계산부 — 메시지 만들기·풀기(protocol), 노드 계산부(bridge), 가짜 모터(fake). ROS·소켓 없이."""
import math
import time

import numpy as np
import pytest

from amr_mpc.ak40 import protocol as P
from amr_mpc.ak40.bridge import ERR_NO_REPLY, MODE_DAMPING, MODE_TORQUE, Ak40Bridge
from amr_mpc.ak40.fake import FakeAk40Motors
from amr_mpc.core import config

LIM = P.MitLimits(p_max=12.5, v_max=45.5, t_max=5.0, kp_max=500.0, kd_max=5.0)   # AK40-10, 매뉴얼 v1.0.18 63쪽
IDS, DIR = [1, 2, 3, 4], [1, -1, 1, -1]


def bridge(**kw):
    a = dict(can_ids=IDS, direction=DIR, limits=LIM, damping_kd=0.02, tau_clamp=4.1, cmd_timeout=0.2, reply_timeout=0.05)
    a.update(kw)
    return Ak40Bridge(**a)


# ── protocol ───────────────────────────────────────────────

def test_command_roundtrip_within_half_step():
    for p, v, kp, kd, t in [(0.0, 0.0, 0.0, 0.0, 0.0), (3.2, -7.5, 12.0, 0.02, 0.0667), (-12.5, 45.5, 500.0, 5.0, -5.0)]:
        got = P.unpack_command(LIM, P.pack_command(LIM, p, v, kp, kd, t))
        for name, x, y in zip(("p", "v", "kp", "kd", "t"), (p, v, kp, kd, t), got):
            assert abs(x - y) <= 0.5 * LIM.step(name) + 1e-12, name


def test_command_byte_layout():
    """Mini Cheetah 규약 배치: p 16 | v 12 | kp 12 | kd 12 | t 12."""
    d = P.pack_command(LIM, 12.5, 45.5, 0.0, 5.0, -5.0)     # p, v 최댓값, kp 0, kd 최댓값, t 최솟값
    assert d == bytes([0xFF, 0xFF, 0xFF, 0xF0, 0x00, 0xFF, 0xF0, 0x00])


def test_out_of_range_is_clamped():
    p, v, kp, kd, t = P.unpack_command(LIM, P.pack_command(LIM, 99.0, -99.0, 9e9, -1.0, 50.0))
    assert p == pytest.approx(12.5) and v == pytest.approx(-45.5) and kp == pytest.approx(500.0)
    assert kd == pytest.approx(0.0) and t == pytest.approx(5.0)


def test_torque_rounding_is_unbiased():
    """원본 C 의 버림이면 평균 −반 칸 치우친다. 반올림이라 평균 오차가 0 근처."""
    taus = np.linspace(-0.1, 0.1, 2001)
    err = [P.unpack_command(LIM, P.pack_command(LIM, 0, 0, 0, 0, x))[4] - x for x in taus]
    assert abs(np.mean(err)) < 0.05 * LIM.step("t")
    assert np.max(np.abs(err)) <= 0.5 * LIM.step("t") + 1e-12


def test_special_frames():
    assert P.ENTER_MOTOR_MODE == bytes.fromhex("FFFFFFFFFFFFFFFC")
    assert P.EXIT_MOTOR_MODE == bytes.fromhex("FFFFFFFFFFFFFFFD")
    assert P.SET_ZERO == bytes.fromhex("FFFFFFFFFFFFFFFE")


@pytest.mark.parametrize("length", [6, 8])
def test_reply_roundtrip(length):
    d = P.pack_reply(LIM, 3, 1.234, -20.5, 0.42, temp_c=37.0, error=5, length=length)
    r = P.unpack_reply(LIM, d)
    assert r["id"] == 3
    assert r["p"] == pytest.approx(1.234, abs=LIM.step("p"))
    assert r["v"] == pytest.approx(-20.5, abs=LIM.step("v"))
    assert r["t"] == pytest.approx(0.42, abs=LIM.step("t"))
    if length == 8:
        assert r["temp"] == 37.0 and r["error"] == 5
    else:
        assert math.isnan(r["temp"]) and r["error"] == 0


def test_unwrapper_across_wrap():
    u = P.PositionUnwrapper(LIM)
    truth = np.arange(0.0, 60.0, 0.4)                       # 2.4 바퀴 넘게 한 방향 (40 rad/s @ 100 Hz)
    got = [u.update(P.wrap_position(x, LIM)) for x in truth]
    assert np.allclose(got, truth)
    u2 = P.PositionUnwrapper(LIM)
    got2 = [u2.update(P.wrap_position(x, LIM)) for x in -truth]
    assert np.allclose(got2, -truth)


# ── bridge (ak40_can_node 계산부) ─────────────────────────────

def test_torque_command_applies_direction_and_clamp():
    b = bridge()
    b.set_command(MODE_TORQUE, [0.05, 0.05, 9.0, -0.05], now=1.0)
    frames, mode, tau_w = b.command_frames(1.01)
    assert mode == MODE_TORQUE and [f[0] for f in frames] == IDS
    t_motor = [P.unpack_command(LIM, d)[4] for _, d in frames]
    assert t_motor == pytest.approx([0.05, -0.05, 4.1, 0.05], abs=LIM.step("t"))   # FR·RR 부호 반전, RL 은 4.1 로 자름
    kp_kd = [P.unpack_command(LIM, d)[2:4] for _, d in frames]
    assert all(kp == 0 and kd == 0 for kp, kd in kp_kd)


def test_damping_on_mode_timeout_and_no_command():
    b = bridge()
    frames, mode, _ = b.command_frames(0.0)                   # 명령 없음
    assert mode == MODE_DAMPING
    p, v, kp, kd, t = P.unpack_command(LIM, frames[0][1])
    assert kp == 0 and v == pytest.approx(0, abs=LIM.step("v")) and kd == pytest.approx(0.02, abs=LIM.step("kd")) \
        and t == pytest.approx(0, abs=LIM.step("t"))
    b.set_command(MODE_TORQUE, [0.1] * 4, now=1.0)
    assert b.command_frames(1.19)[1] == MODE_TORQUE
    assert b.command_frames(1.21)[1] == MODE_DAMPING          # cmd_timeout 0.2 s 초과
    b.set_command(MODE_DAMPING, [0.1] * 4, now=2.0)
    assert b.command_frames(2.0)[1] == MODE_DAMPING
    b.set_command(MODE_TORQUE, [0.1] * 4, now=3.0)
    assert b.command_frames(3.0, force_damping=True)[1] == MODE_DAMPING


def test_reply_maps_to_wheels_with_direction():
    b = bridge()
    b.start_frames(0.0)
    for cid, v in zip(IDS, [10.0, -10.0, 10.0, -10.0]):     # 모터 기준 — 바퀴 기준으로는 4개 모두 +10 (전진)
        assert b.on_reply(P.pack_reply(LIM, cid, 0.5, v, 0.03 * np.sign(v)), now=0.01) is not None
    s = b.wheel_states(0.01)
    assert s["velocity"] == pytest.approx([10.0] * 4, abs=LIM.step("v"))
    assert s["torque"] == pytest.approx([0.03] * 4, abs=LIM.step("t"))
    assert s["position"] == pytest.approx([0.5, -0.5, 0.5, -0.5], abs=LIM.step("p"))
    assert s["fresh"].all() and s["stamp"] == 0.01
    assert not b.wheel_states(0.02)["fresh"].any()          # 다음 호출에는 새 응답 없음
    assert b.on_reply(P.pack_reply(LIM, 9, 0, 0, 0), now=0.02) is None   # 모르는 ID


def test_missing_reply_sets_error():
    b = bridge()
    b.start_frames(0.0)
    for cid in IDS[:3]:
        b.on_reply(P.pack_reply(LIM, cid, 0, 0, 0), now=0.08)
    s = b.wheel_states(0.09)
    assert list(s["error"]) == [0, 0, 0, ERR_NO_REPLY]      # RR 은 0.05 s 넘게 응답 없음


def test_no_error_right_after_start():
    b = bridge()
    b.start_frames(0.0)
    assert not b.wheel_states(0.01)["error"].any()           # 시작 직후 reply_timeout 안은 봐준다


def test_bridge_rejects_bad_config():
    with pytest.raises(ValueError):
        bridge(can_ids=[1, 1, 2, 3])
    with pytest.raises(ValueError):
        bridge(direction=[1, 0, 1, -1])


# ── 가짜 모터 + bridge 왕복 ───────────────────────────────────

def loop(b, fake, wheel_pos, wheel_vel, wheel_tau, now):
    """노드 한 주기: 명령 송신 → 가짜 모터 응답 → 노드 반영."""
    frames, mode, _ = b.command_frames(now)
    for cid, d in frames:
        r = fake.on_frame(cid, d, wheel_pos, wheel_vel, wheel_tau)
        b.on_reply(r, now)
    return mode


def test_fake_motor_roundtrip_signs():
    """노드가 바퀴 +0.05 N·m 를 보내면, 장착 방향이 섞여 있어도 4 바퀴 모두 +0.05 를 받고 상태도 바퀴 기준으로 돌아온다."""
    b, fake = bridge(), FakeAk40Motors(IDS, DIR, LIM, 4.1)
    for cid, d in b.start_frames(0.0):
        fake.on_frame(cid, d, np.zeros(4), np.zeros(4), np.zeros(4))
    b.set_command(MODE_TORQUE, [0.05] * 4, now=0.0)
    loop(b, fake, np.zeros(4), np.zeros(4), np.zeros(4), 0.0)
    assert fake.torque(np.zeros(4), np.zeros(4)) == pytest.approx([0.05] * 4, abs=LIM.step("t"))
    wp, wv = np.array([1.0, 1.0, 1.0, 1.0]), np.array([12.0, 12.0, 12.0, 12.0])
    loop(b, fake, wp, wv, np.full(4, 0.05), 0.01)
    s = b.wheel_states(0.01)
    assert s["velocity"] == pytest.approx(wv, abs=LIM.step("v")) and s["position"] == pytest.approx(wp, abs=LIM.step("p"))
    assert s["torque"] == pytest.approx([0.05] * 4, abs=LIM.step("t"))


def test_fake_damping_brakes_wheels():
    b, fake = bridge(), FakeAk40Motors(IDS, DIR, LIM, 4.1)
    for cid, d in b.start_frames(0.0):
        fake.on_frame(cid, d, np.zeros(4), np.zeros(4), np.zeros(4))
    wv = np.array([20.0, -5.0, 3.0, 0.0])
    loop(b, fake, np.zeros(4), wv, np.zeros(4), 0.0)       # 명령 없음 → DAMPING
    _, v_q, _, kd_q, t_q = P.unpack_command(LIM, P.pack_command(LIM, 0, 0, 0, 0.02, 0))
    d = np.array(DIR, float)
    expect = d * (kd_q * (v_q - d * wv) + t_q)              # 모터가 실제로 받는 (잘린) 값으로 계산
    assert fake.torque(np.zeros(4), wv) == pytest.approx(expect, abs=1e-9)
    assert fake.torque(np.zeros(4), wv)[0] == pytest.approx(-0.02 * 20.0, rel=0.05)   # 대략 −kd·ω


def test_zero_is_not_representable():
    """12 비트 ±범위에서 0 은 정수로 안 떨어진다 (4095 칸의 가운데 2047.5). 반올림하면 반 칸 위로 간다.
    → AK40-10 (T ±5, V ±45.5, KD 5): "토크 0" 은 +1.22 mN·m, "속도 0" 은 +0.011 rad/s, kd 0.02 는 0.0195.
    DAMPING 으로 세워도 무부하 바퀴는 (t + kd·v_des)/kd ≈ 0.07 rad/s 로 기어갈 수 있다 — 벤치에서 확인할 것."""
    _, v_q, _, kd_q, t_q = P.unpack_command(LIM, P.pack_command(LIM, 0, 0, 0, 0.02, 0))
    assert t_q == pytest.approx(0.5 * LIM.step("t")) and v_q == pytest.approx(0.5 * LIM.step("v"))
    assert kd_q == pytest.approx(0.0195, abs=1e-4)
    creep = (t_q + kd_q * v_q) / kd_q
    assert 0.05 < creep < 0.1


def test_fake_holds_last_command_and_exit_frees():
    """진짜 모터처럼 명령이 끊겨도 마지막 토크를 계속 낸다 (그래서 끊김 처리는 노드 몫). 모드에서 나오면 0."""
    b, fake = bridge(), FakeAk40Motors(IDS, DIR, LIM, 4.1)
    for cid, d in b.start_frames(0.0):
        fake.on_frame(cid, d, np.zeros(4), np.zeros(4), np.zeros(4))
    b.set_command(MODE_TORQUE, [0.1] * 4, now=0.0)
    loop(b, fake, np.zeros(4), np.zeros(4), np.zeros(4), 0.0)
    assert fake.torque(np.zeros(4), np.zeros(4)) == pytest.approx([0.1] * 4, abs=LIM.step("t"))   # 다음 명령 전까지 유지
    for cid, d in b.stop_frames():
        fake.on_frame(cid, d, np.zeros(4), np.zeros(4), np.zeros(4))
    assert np.all(fake.torque(np.zeros(4), np.ones(4)) == 0.0)


def test_small_torque_quantization_for_group_c():
    """비교군 C 의 상한 0.0667 N·m 근처에서 한 칸 크기 (AK40-10 T ±5): 3.7 % — 측정·보고용 숫자를 고정해 둔다."""
    step = LIM.step("t")
    assert step == pytest.approx(10.0 / 4095)
    assert 0.03 < step / 0.0667 < 0.04


def test_max_does_not_overflow():
    """매뉴얼 보내는 쪽 예제 (x − min)·2^bits/span 은 최댓값에서 4096 이 돼 12 비트를 넘친다(→ 0). 우리는 4095 로 자른다."""
    assert P.float_to_uint(5.0, -5.0, 5.0, 12) == 4095
    assert P.float_to_uint(-5.0, -5.0, 5.0, 12) == 0
    assert int((5.0 + 5.0) * (4096 / 10.0)) == 4096          # 예제 식 그대로면 넘친다


def test_reply_id_rule():
    """매뉴얼 61쪽: 응답 CAN ID = 0X00 + Drive ID (모터 자기 ID). [0] 도 모터 ID."""
    b = bridge()
    d = P.pack_reply(LIM, 3, 0, 0, 0)
    assert b.is_reply(3, d) and not b.is_reply(0, d) and not b.is_reply(2, d)
    assert not b.is_reply(3, P.pack_command(LIM, 0, 0, 0, 0, 0))   # 명령 프레임([0] = 위치 상위 바이트 0x7F/0x80)은 아님
    b0 = bridge(reply_can_id=0)
    assert b0.is_reply(0, d) and not b0.is_reply(3, d)
    fake = FakeAk40Motors(IDS, DIR, LIM, 4.1)
    assert fake.reply_id(3) == 3


def test_first_run_clamp():
    """hardware_checked false 인 첫 시험: 노드가 tau_clamp 를 0.1 로 주면 큰 명령도 0.1 로 잘린다."""
    b = bridge(tau_clamp=0.1)
    b.set_command(MODE_TORQUE, [1.0, -1.0, 0.05, 0.0], now=0.0)
    t_motor = [P.unpack_command(LIM, d)[4] for _, d in b.command_frames(0.0)[0]]
    assert t_motor == pytest.approx([0.1, 0.1, 0.05, 0.0], abs=LIM.step("t"))   # FR 은 부호 반전 (−1 × −1.0 → +0.1)


def test_reenter_motor_that_missed_start():
    """시작 때 ENTER 를 못 받은 모터(늦게 켜짐)는 응답이 없다 → 노드가 ENTER 를 다시 보내 켠다 (2026-11-12 시뮬에서 발견)."""
    b, fake = bridge(), FakeAk40Motors(IDS, DIR, LIM, 4.1)
    start = b.start_frames(0.0)
    for cid, d in start[:2]:                                 # RL·RR 은 시작 ENTER 를 못 받음
        fake.on_frame(cid, d, np.zeros(4), np.zeros(4), np.zeros(4))
    b.set_command(MODE_TORQUE, [0.05] * 4, now=0.0)
    for k in range(40):                                      # 0.4 s, 100 Hz
        now = 0.01 * k
        b.set_command(MODE_TORQUE, [0.05] * 4, now=now)
        for cid, d in b.reenter_frames(now) + b.command_frames(now)[0]:
            r = fake.on_frame(cid, d, np.zeros(4), np.zeros(4), np.zeros(4))
            if r is not None:
                b.on_reply(r, now)
        b.wheel_states(now)
    assert fake.enabled.all()
    assert fake.torque(np.zeros(4), np.zeros(4)) == pytest.approx([0.05] * 4, abs=LIM.step("t"))


def test_disabled_fake_ignores_commands():
    fake = FakeAk40Motors(IDS, DIR, LIM, 4.1)
    assert fake.on_frame(1, P.pack_command(LIM, 0, 0, 0, 0, 0.1), np.zeros(4), np.zeros(4), np.zeros(4)) is None
    assert np.all(fake.torque(np.zeros(4), np.zeros(4)) == 0.0)


def test_recorded_rr_reply_matches_identification_csv():
    # 20261111_183226_841201_BENCH/samples.csv, cycle 0 (실제 수신 payload).
    payload = bytes.fromhex('04c8ad7fd7ff4800')
    r = P.unpack_reply(LIM, payload)
    assert r == pytest.approx(dict(id=4, p=7.097543297474633,
                                  v=-0.055555555555557135,
                                  t=-0.0012210012210012167, temp=32, error=0))


def test_quantized_torque_stays_inside_first_run_limit():
    b = bridge(tau_clamp=0.1)
    for requested in np.linspace(-0.2, 0.2, 401):
        b.set_command(MODE_TORQUE, [requested] * 4, now=0)
        for _, payload in b.command_frames(0)[0]:
            assert abs(P.unpack_command(LIM, payload)[4]) <= 0.1


@pytest.mark.parametrize('torque', [[1, 2, 3], [0, 0, 0, math.nan], [0, 0, math.inf, 0]])
def test_invalid_command_replaces_previous_torque_with_damping(torque):
    b = bridge()
    b.set_command(MODE_TORQUE, [0.1] * 4, now=0)
    b.set_command(MODE_TORQUE, torque, now=0.01)
    assert b.command_frames(0.01)[1] == MODE_DAMPING


def test_configured_reply_length_requires_temperature_and_error():
    b = bridge()
    payload = P.pack_reply(LIM, 1, 0, 0, 0)
    assert b.is_reply(1, payload)
    assert not b.is_reply(1, payload[:6])
    assert not b.is_reply(1, payload[:7])
    assert bridge(reply_length=6).is_reply(1, payload[:6])


@pytest.mark.parametrize('flags', [0x80000000, 0x40000000, 0x20000000])
def test_socket_bus_rejects_flagged_frames(flags):
    import struct
    from amr_mpc.ak40.bus import CanBus

    class Socket:
        def recv(self, size):
            return struct.pack('=IB3x8s', flags | 1, 8, P.pack_reply(LIM, 1, 0, 0, 0))

    bus = CanBus.__new__(CanBus)
    bus.sock = Socket()
    assert bus.recv() is None


def test_hardware_config_clockwise_positive_motor_mounting():
    from pathlib import Path
    import yaml

    c = yaml.safe_load((Path(__file__).parents[1] / 'config/ak40.yaml').read_text())['ak40']
    # 출력축 바깥에서 +가 시계 방향: 전진하려면 왼쪽 -, 오른쪽 +.
    b = bridge(can_ids=c['can_ids'], direction=c['direction'])
    b.set_command(MODE_TORQUE, [0.05] * 4, now=0)
    motor_tau = [P.unpack_command(LIM, data)[4] for _, data in b.command_frames(0)[0]]
    assert motor_tau == pytest.approx([-0.05, 0.05, -0.05, 0.05], abs=LIM.step('t'))
    for cid, velocity in zip(c['can_ids'], [-2, 2, -2, 2]):
        b.on_reply(P.pack_reply(LIM, cid, 0, velocity, 0), now=0.01)
    assert b.wheel_states(0.01)['velocity'] == pytest.approx([2] * 4, abs=LIM.step('v'))


# ── R8 리뷰 (2026-11-14): 실차에서만 드러나는 것 ────────────────────────────────────
class _LateBus:
    """응답이 늘 reply_wait 를 넘겨 오는 버스 (실차 slcan 흉내). 보낸 명령의 응답은 기다리는 동안엔 안 오고,
    다음 주기 시작(기다리지 않는 recv(0))에야 보인다."""
    def __init__(self, ifname):
        from amr_mpc.ak40.fake import FakeAk40Motors
        c = config.load("ak40.yaml")["ak40"]
        self.fake = FakeAk40Motors(c["can_ids"], c["direction"], P.MitLimits.from_dict(c["mit_limits"]), 4.1)
        self.pending = []

    def send(self, can_id, data):
        r = self.fake.on_frame(can_id, data, np.zeros(4), np.full(4, 2.0), np.zeros(4))   # 바퀴 2 rad/s 로 도는 중
        if r is not None:
            self.pending.append((can_id, r))

    def recv(self, timeout=0.0):
        if timeout > 0.0:                    # reply_wait 안에는 아무것도 안 온다
            time.sleep(min(timeout, 0.001))
            return None
        return self.pending.pop(0) if self.pending else None

    def close(self):
        pass


def test_late_replies_are_used_not_dropped(monkeypatch):
    """응답이 늘 늦어도 다음 주기에 반영해, 네 바퀴가 'error 255(응답 없음)' 로 바뀌지 않는다.
    예전(bus.drain 으로 버림)에는 reply_timeout 0.05 s 뒤 네 바퀴 모두 255 → fault_monitor 가 무장을 풀어 로봇이 못 움직였다."""
    import rclpy
    import amr_mpc.nodes.ak40_can as node_mod
    monkeypatch.setattr(node_mod, "CanBus", _LateBus)
    rclpy.init(args=["--ros-args", "-p", "interface:=vcan_late_test", "-p", "cmd_topic:=/r8_test/cmd",
                     "-p", "wheel_states_topic:=/r8_test/ws"])
    try:
        n = node_mod.Ak40CanNode()
        published = []
        n.pub_ws.publish = published.append
        n.pub_status.publish = lambda m: None
        for _ in range(30):                  # 약 0.15 s > reply_timeout 0.05 s
            n.on_tick()
        last = published[-1]
        assert list(last.error) == [0, 0, 0, 0]
        assert np.allclose(last.velocity, 2.0, atol=0.05)          # 바퀴 상태는 늦은 응답으로 계속 갱신된다
        assert n.late >= 4 * 29
        assert list(last.fresh) == [True] * 4                       # 늦은 응답도 이번에 받은 것으로 센다 → 거짓 "응답 0/4" 경고 없음
        assert n.missing <= 4                                       # 첫 주기(아직 아무 응답 없음)만
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_sigterm_runs_damping_then_exit(tmp_path):
    """kill(SIGTERM) 으로 꺼도 Ctrl+C 와 같이 DAMPING 을 보낸 뒤 모터 모드에서 나온다.
    예전에는 SIGTERM 에 바로 죽어 모터가 마지막 명령을 계속 낼 수 있었다."""
    import os
    import signal
    import subprocess
    import sys
    log = tmp_path / "frames.txt"
    code = f'''
import sys
import amr_mpc.nodes.ak40_can as m
class RecBus:
    def __init__(self, ifname):
        self.f = open({str(log)!r}, "a", buffering=1)
    def send(self, can_id, data):
        self.f.write(f"{{can_id}} {{bytes(data).hex()}}\\n")
    def recv(self, timeout=0.0):
        return None
    def close(self):
        self.f.close()
m.CanBus = RecBus
sys.argv = ["ak40_can_node", "--ros-args", "-p", "interface:=vcan_sigterm_test",
            "-p", "cmd_topic:=/r8_sig/cmd", "-p", "wheel_states_topic:=/r8_sig/ws"]
m.main()
'''
    env = dict(os.environ, ROS_DOMAIN_ID="93")
    p = subprocess.Popen([sys.executable, "-c", code], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        end = time.monotonic() + 15.0
        while time.monotonic() < end and (not log.exists() or log.read_text().count("\n") < 40):
            time.sleep(0.1)                  # 기동 + 주기 송신이 시작될 때까지
        assert log.exists(), p.stdout.read().decode() if p.poll() is not None else "기동 안 됨"
        p.send_signal(signal.SIGTERM)
        out = p.communicate(timeout=10)[0].decode()
    finally:
        if p.poll() is None:
            p.kill()
    lines = log.read_text().split()
    frames = list(zip(lines[0::2], lines[1::2]))
    exit_hex = P.EXIT_MOTOR_MODE.hex()
    assert [d for _, d in frames[-4:]] == [exit_hex] * 4, out          # 마지막은 네 모터 모두 모터 모드 나오기
    assert "DAMPING 뒤 모터 모드 해제" in out


def test_sim_plant_fake_can_mode_starts(monkeypatch):
    """plant:=sim_can 의 시뮬 플랜트가 가짜 모터 모드로 켜진다 (R8 리뷰: protocol import 가 빠져 켜지자마자 죽던 것).
    실제 vcan 없이 CanBus 만 바꿔 생성자를 돈다."""
    import rclpy
    import amr_mpc.nodes.sim_plant_torque as sim_mod

    class NullBus:
        def __init__(self, ifname):
            pass

        def recv(self, timeout=0.0):
            time.sleep(min(timeout, 0.01))
            return None

        def send(self, can_id, data):
            pass

        def close(self):
            pass

    monkeypatch.setattr(sim_mod, "CanBus", NullBus)
    rclpy.init(args=["--ros-args", "-p", "can_interface:=vcan_test", "-p", "can_reply_delay:=0.006",
                     "-p", "imu_topic:=/r8_sim/imu", "-p", "pose_topic:=/r8_sim/pose",
                     "-p", "wheel_states_topic:=/r8_sim/ws", "-p", "truth_topic:=/r8_sim/truth",
                     "-p", "cmd_topic:=/r8_sim/cmd"])
    try:
        n = sim_mod.SimPlantTorqueNode()
        assert n.fake is not None and n.can_delay == 0.006
        n.destroy_node()
    finally:
        rclpy.shutdown()
