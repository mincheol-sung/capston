#!/usr/bin/env python3
"""AK40-10 MIT 모드 CAN 메시지 — 만들기·풀기. ROS·소켓에 의존하지 않는다 (단위 시험 test/test_ak40_protocol.py).

근거: CubeMars AK 시리즈 드라이버 매뉴얼 v1.0.18 (AK 2.0), 5.3 MIT Communication Protocol 60~68쪽
      (amr/notes_amr/ak-series-driver-manual-v1-0-18-for-ak-2-0-robotic-actuator.pdf). 2026-11-12 대조 — 아래 배치가 매뉴얼과 같다.
MIT Mini Cheetah 드라이버(Ben Katz) 규약. 11 비트 표준 ID, 8 바이트, 1 Mbps.
범위(P/V/T/KP/KD 최댓값)는 모델마다 다르다 → config/ak40.yaml mit_limits (AK40-10: P ±12.5, V ±45.5, T ±5).

특수 명령 (ID = 모터 ID)
  모터 모드 들어가기  FF FF FF FF FF FF FF FC     나오기  … FD     현재 위치를 0 으로  … FE

명령 (ID = 모터 ID) — 모터는 τ = kp·(p_des − p) + kd·(v_des − v) + t_ff 를 낸다
  p_des 16 비트 | v_des 12 | kp 12 | kd 12 | t_ff 12
  [0] p>>8  [1] p&FF  [2] v>>4  [3] (v&F)<<4 | kp>>8  [4] kp&FF  [5] kd>>4  [6] (kd&F)<<4 | t>>8  [7] t&FF
  우리 쓰임:  MODE_TORQUE  → kp = 0, kd = 0, t_ff = τ        (WheelTorqueCommand.msg)
              MODE_DAMPING → kp = 0, v_des = 0, kd = damping_kd, t_ff = 0

응답 (모터가 명령마다 하나. CAN ID 는 "0X00 + Drive ID" = 모터 자기 ID, [0] 도 모터 ID)
  [0] 모터 ID  [1..2] p 16 비트  [3] v>>4  [4] (v&F)<<4 | t>>8  [5] t&FF  [6] 온도 (+오프셋)  [7] 에러 코드
  6 바이트 펌웨어는 [6][7] 이 없다 (온도 NaN, 에러 0 으로 둔다).
  에러 코드 (매뉴얼 5.2 절 응답 정의): 0 정상, 1 모터 과열, 2 과전류, 3 과전압, 4 저전압, 5 엔코더, 6 MOSFET 과열

정수 변환: 매뉴얼의 보내는 쪽 예제는 (x − min)·2^bits/span 을 버림하고, 받는 쪽 예제(uint_to_float)는 span/(2^bits − 1) 로 푼다.
  우리는 받는 쪽 식의 정확한 역(×(2^bits − 1)/span, 반올림)으로 보낸다 — 예제대로 하면 최댓값이 2^bits 로 넘쳐 0 이 되고
  (12 비트 4096 → 0), 버림 때문에 평균 반 칸 치우친다. 어느 쪽이든 12 비트 ±범위에서 0 은 정확히 표현되지 않는다
  (4095 칸의 가운데). AK40-10 (T ±5) 이면 토크 한 칸 2.44 mN·m, "0" 은 +1.22 mN·m 로 간다.
"""
from dataclasses import dataclass
import math

ENTER_MOTOR_MODE = bytes([0xFF] * 7 + [0xFC])
EXIT_MOTOR_MODE = bytes([0xFF] * 7 + [0xFD])
SET_ZERO = bytes([0xFF] * 7 + [0xFE])
SPECIAL = {ENTER_MOTOR_MODE: "enter", EXIT_MOTOR_MODE: "exit", SET_ZERO: "zero"}


@dataclass(frozen=True)
class MitLimits:
    p_max: float = 12.5      # [rad]   위치 ±      (AK40-10, 매뉴얼 63쪽)
    v_max: float = 45.5      # [rad/s] 속도 ±
    t_max: float = 5.0       # [N·m]   토크 ±
    kp_max: float = 500.0    # kp 0 … 이 값
    kd_max: float = 5.0      # kd 0 … 이 값

    @classmethod
    def from_dict(cls, d):
        return cls(**{k: float(d[k]) for k in cls.__dataclass_fields__ if k in d})

    def step(self, name):
        """그 필드 정수 한 칸의 크기 (12/16 비트)."""
        span, bits = {"p": (2 * self.p_max, 16), "v": (2 * self.v_max, 12), "t": (2 * self.t_max, 12),
                      "kp": (self.kp_max, 12), "kd": (self.kd_max, 12)}[name]
        return span / ((1 << bits) - 1)


ERROR_NAMES = {0: "정상", 1: "모터 과열", 2: "과전류", 3: "과전압", 4: "저전압", 5: "엔코더", 6: "MOSFET 과열"}


def float_to_uint(x, lo, hi, bits):
    """[lo, hi] → 0 … 2^bits − 1, 반올림. 범위 밖은 끝으로 자른다."""
    x = min(max(float(x), lo), hi)
    n = (1 << bits) - 1
    return int(round((x - lo) * n / (hi - lo)))


def uint_to_float(u, lo, hi, bits):
    return lo + (hi - lo) * float(u) / ((1 << bits) - 1)


def pack_command(lim, p, v, kp, kd, t, torque_limit=None):
    """MIT 명령 8 바이트."""
    pi = float_to_uint(p, -lim.p_max, lim.p_max, 16)
    vi = float_to_uint(v, -lim.v_max, lim.v_max, 12)
    kpi = float_to_uint(kp, 0.0, lim.kp_max, 12)
    kdi = float_to_uint(kd, 0.0, lim.kd_max, 12)
    ti = float_to_uint(t, -lim.t_max, lim.t_max, 12)
    if torque_limit is not None:
        # 벤치 코드 pack(torque, limit)와 같이 양자화 후에도 상한을 지킨다.
        bound = min(float(torque_limit), lim.t_max)
        if not math.isfinite(bound) or bound < lim.step("t") / 2:
            raise ValueError("torque_limit must allow at least half a torque step")
        low = math.ceil((lim.t_max - bound) / lim.step("t"))
        high = math.floor((lim.t_max + bound) / lim.step("t"))
        ti = min(max(ti, low), high)
    return bytes([pi >> 8, pi & 0xFF, vi >> 4, ((vi & 0xF) << 4) | (kpi >> 8), kpi & 0xFF,
                  kdi >> 4, ((kdi & 0xF) << 4) | (ti >> 8), ti & 0xFF])


def unpack_command(lim, data):
    """pack_command 의 역 (가짜 모터가 쓴다). 반환 (p, v, kp, kd, t)."""
    d = data
    pi = (d[0] << 8) | d[1]
    vi = (d[2] << 4) | (d[3] >> 4)
    kpi = ((d[3] & 0xF) << 8) | d[4]
    kdi = (d[5] << 4) | (d[6] >> 4)
    ti = ((d[6] & 0xF) << 8) | d[7]
    return (uint_to_float(pi, -lim.p_max, lim.p_max, 16), uint_to_float(vi, -lim.v_max, lim.v_max, 12),
            uint_to_float(kpi, 0.0, lim.kp_max, 12), uint_to_float(kdi, 0.0, lim.kd_max, 12),
            uint_to_float(ti, -lim.t_max, lim.t_max, 12))


def pack_reply(lim, motor_id, p, v, t, temp_c=25.0, error=0, length=8, temp_offset=40):
    """모터 응답 (가짜 모터가 쓴다)."""
    pi = float_to_uint(p, -lim.p_max, lim.p_max, 16)
    vi = float_to_uint(v, -lim.v_max, lim.v_max, 12)
    ti = float_to_uint(t, -lim.t_max, lim.t_max, 12)
    out = [motor_id & 0xFF, pi >> 8, pi & 0xFF, vi >> 4, ((vi & 0xF) << 4) | (ti >> 8), ti & 0xFF]
    if length >= 8:
        out += [int(min(max(round(temp_c + temp_offset), 0), 255)), int(error) & 0xFF]
    return bytes(out)


def unpack_reply(lim, data, temp_offset=40):
    """모터 응답 → dict(id, p, v, t, temp, error). 6 바이트면 temp NaN, error 0."""
    d = data
    if len(d) < 6:
        raise ValueError(f"응답이 {len(d)} 바이트 — 6 이상이어야 한다")
    pi = (d[1] << 8) | d[2]
    vi = (d[3] << 4) | (d[4] >> 4)
    ti = ((d[4] & 0xF) << 8) | d[5]
    return {
        "id": d[0],
        "p": uint_to_float(pi, -lim.p_max, lim.p_max, 16),
        "v": uint_to_float(vi, -lim.v_max, lim.v_max, 12),
        "t": uint_to_float(ti, -lim.t_max, lim.t_max, 12),
        "temp": float(d[6] - temp_offset) if len(d) >= 8 else math.nan,
        "error": int(d[7]) if len(d) >= 8 else 0,
    }


def wrap_position(p, lim):
    """모터가 내는 위치는 ±p_max 에서 돈다 (가짜 모터용)."""
    span = 2.0 * lim.p_max
    return (p + lim.p_max) % span - lim.p_max


class PositionUnwrapper:
    """±p_max 에서 도는 위치를 펼친 누적값으로. 한 주기 변화가 반 바퀴(p_max)보다 크면 넘어간 것으로 본다.
    100 Hz 에서 p_max 12.5 rad 면 1250 rad/s 까지 안전 (모터 최고속도 약 42 rad/s)."""

    def __init__(self, lim):
        self.span = 2.0 * lim.p_max
        self.half = lim.p_max
        self.last = None
        self.turns = 0

    def update(self, p):
        if self.last is not None:
            dp = p - self.last
            if dp > self.half:
                self.turns -= 1
            elif dp < -self.half:
                self.turns += 1
        self.last = p
        return p + self.turns * self.span
