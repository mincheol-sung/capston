"""가짜 RoboClaw — Basicmicro API 의 일부를 같은 이름·같은 반환 형식으로 흉내 낸다.

roboclaw_probe(통신 시간·지연 분해 측정)와 드라이버 시험이 하드웨어 없이 돌도록 쓴다.
알고 있는 값을 심어 두고, 측정·분석 도구가 그 값을 되찾는지 검증하는 것이 목적이다.

모터 (채널마다):
    명령 qpps 는 act_delay 뒤에 목표가 되고, 속도는 시정수 tau 의 1차로 따라간다.
    엔코더 카운트 = 속도의 적분 (정수로 잘라 32-bit two's complement 로 낸다).
읽기 값:
    ReadSpeedM1/M2 (cmd 18/19)  : 최근 speed_window 초 동안의 카운트 변화 / speed_window  [counts/s]
    ReadISpeedM1/M2 (cmd 30/31) : 최근 1/125 s 창의 속도 [counts/s] (2026-10-22 실측: 명령 1681 에 원시값 ≈ 1800, 잡음 큼)
    GetSpeeds (cmd 108)         : cmd 18 두 채널을 한 번에. supports_getspeeds=False 면 미지원
    GetEncoders (cmd 78)        : 두 채널 카운트
통신:
    호출마다 latency 초(균일분포 [latency_min, latency_max])를 포트 잠금을 쥔 채 보낸다.
    장치가 요청을 처리하는 순간은 그 가운데다.  fail_rate 확률로 timeout×retries 를 쓰고 예외.
"""
from __future__ import annotations

import random
import threading
import time
from collections import deque
from typing import Dict, Optional

try:
    from basicmicro.exceptions import PacketTimeoutError
except ImportError:  # pragma: no cover - basicmicro 가 없는 환경
    class PacketTimeoutError(Exception):
        pass

_DT = 0.00025                     # 물리 적분 간격 [s]
_ISPEED_WINDOW = 1.0 / 125.0       # cmd 30/31 정의


def _u32(v: int) -> int:
    return int(v) & 0xFFFFFFFF


class _Motor:
    def __init__(self, tau: float, act_delay: float, gain: float, speed_window: float, t0: float):
        self.tau, self.act_delay, self.gain, self.speed_window = tau, act_delay, gain, speed_window
        self.speed = 0.0                      # [counts/s]
        self.count = 0.0
        self.target = 0.0
        self.pending: deque = deque()         # (t_effective, qpps)
        self.t = t0
        self.hist: deque = deque([(t0, 0.0)])  # (t, count) — 창 속도 계산용
        self.pwm = 0.0

    def command(self, t: float, qpps: int):
        self.pending.append((t + self.act_delay, float(qpps)))

    def advance(self, t: float):
        while self.t + _DT <= t:
            self.t += _DT
            while self.pending and self.pending[0][0] <= self.t:
                self.target = self.pending.popleft()[1]
            a = _DT / self.tau
            self.speed += a * (self.gain * self.target - self.speed)
            self.count += self.speed * _DT
            self.hist.append((self.t, self.count))
            self.pwm = max(-1.0, min(1.0, self.target / 5000.0))
        horizon = self.t - max(self.speed_window, _ISPEED_WINDOW) - 0.01
        while len(self.hist) > 2 and self.hist[1][0] < horizon:
            self.hist.popleft()

    def _count_at(self, t: float) -> float:
        for tt, c in reversed(self.hist):
            if tt <= t:
                return c
        return self.hist[0][1]

    def window_speed(self, window: float) -> float:
        return (self.count - self._count_at(self.t - window)) / window


class FakeRoboClaw:
    """Basicmicro 대용. 포트 하나 = 보드 하나 = 모터 둘."""

    def __init__(self, comport: str = 'fake', rate: int = 115200, timeout: float = 0.01, retries: int = 2,
                 *, tau: float = 0.09, act_delay: float = 0.015, gain: float = 1.0, speed_window: float = 0.03,
                 latency_min: float = 0.0008, latency_max: float = 0.0014, fail_rate: float = 0.0,
                 supports_getspeeds: bool = True, seed: Optional[int] = None):
        self.comport, self.rate, self.timeout, self.retries = comport, rate, timeout, retries
        self.latency = (latency_min, latency_max)
        self.fail_rate = fail_rate
        self.supports_getspeeds = supports_getspeeds
        self._rng = random.Random(seed)
        self._lock = threading.Lock()
        t0 = time.monotonic()
        self.motors: Dict[int, _Motor] = {1: _Motor(tau, act_delay, gain, speed_window, t0),
                                          2: _Motor(tau, act_delay, gain, speed_window, t0)}
        self.is_open = False
        self.calls = 0

    # ---- 연결 ----
    def Open(self) -> bool:
        self.is_open = True
        return True

    def close(self) -> None:
        self.is_open = False

    # ---- 공통: 지연을 쓰고 가운데 순간에 장치 쪽 일을 한다 ----
    def _transact(self, fn, supported: bool = True):
        with self._lock:
            self.calls += 1
            if not supported or self._rng.random() < self.fail_rate:
                time.sleep(self.timeout * self.retries)
                raise PacketTimeoutError('fake timeout')
            lat = self._rng.uniform(*self.latency)
            time.sleep(lat / 2)
            now = time.monotonic()
            for m in self.motors.values():
                m.advance(now)
            out = fn(now)
            time.sleep(lat / 2)
            return out

    # ---- 쓰기 ----
    def SpeedM1M2(self, address: int, m1: int, m2: int) -> bool:
        def f(now):
            self.motors[1].command(now, m1)
            self.motors[2].command(now, m2)
            return True
        return self._transact(f)

    # ---- 읽기 (Basicmicro 와 같은 튜플) ----
    def ReadVersion(self, address: int):
        return self._transact(lambda now: (True, 'FakeRoboClaw 0.0'))

    def ReadSpeedM1(self, address: int):
        return self._transact(lambda now: (True, _u32(round(self.motors[1].window_speed(self.motors[1].speed_window))), 0))

    def ReadSpeedM2(self, address: int):
        return self._transact(lambda now: (True, _u32(round(self.motors[2].window_speed(self.motors[2].speed_window))), 0))

    def ReadISpeedM1(self, address: int):
        return self._transact(lambda now: (True, _u32(round(self.motors[1].window_speed(_ISPEED_WINDOW))), 0))

    def ReadISpeedM2(self, address: int):
        return self._transact(lambda now: (True, _u32(round(self.motors[2].window_speed(_ISPEED_WINDOW))), 0))

    def GetSpeeds(self, address: int):
        m1, m2 = self.motors[1], self.motors[2]
        return self._transact(
            lambda now: (True, _u32(round(m1.window_speed(m1.speed_window))), _u32(round(m2.window_speed(m2.speed_window)))),
            supported=self.supports_getspeeds)

    def GetEncoders(self, address: int):
        return self._transact(lambda now: (True, _u32(int(self.motors[1].count)), _u32(int(self.motors[2].count))))

    def ReadEncM1(self, address: int):
        return self._transact(lambda now: (True, _u32(int(self.motors[1].count)), 0))

    def ReadEncM2(self, address: int):
        return self._transact(lambda now: (True, _u32(int(self.motors[2].count)), 0))

    def ReadPWMs(self, address: int):
        return self._transact(lambda now: (True, int(self.motors[1].pwm * 32767), int(self.motors[2].pwm * 32767)))

    def ReadCurrents(self, address: int):
        return self._transact(lambda now: (True, 0, 0))

    def ReadMainBatteryVoltage(self, address: int):
        return self._transact(lambda now: (True, 160))

    def ReadM1VelocityPID(self, address: int):
        return self._transact(lambda now: (True, 1.0, 0.5, 0.0, 6000))

    def ReadM2VelocityPID(self, address: int):
        return self._transact(lambda now: (True, 1.0, 0.5, 0.0, 6000))
