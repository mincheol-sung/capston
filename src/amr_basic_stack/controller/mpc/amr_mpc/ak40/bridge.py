#!/usr/bin/env python3
"""ak40_can_node 의 계산부 — 바퀴 토크 명령 ↔ 모터 4개 CAN 프레임. ROS·소켓에 의존하지 않는다.

  바퀴 순서 [FL, FR, RL, RR] (WheelTorqueCommand·WheelStates 규약), 모터는 can_ids 로 대응.
  부호: direction[i] = +1 이면 모터 + 회전 = 바퀴 + 회전(전진 방향). 좌우가 마주 보고 달리면 한쪽이 −1.
        보낼 때 τ_모터 = direction·τ_바퀴, 받을 때 ω_바퀴 = direction·ω_모터 (위치·토크도 같다).
  명령 규칙 (시뮬 플랜트와 같은 규칙 — sim_plant_torque.py 머리주석)
    MODE_TORQUE   → kp 0, kd 0, t_ff = τ (±tau_clamp 로 자름)
    MODE_DAMPING  → kp 0, v_des 0, kd = damping_kd
    명령 없음 / cmd_timeout 초과 → DAMPING. (모터는 마지막 명령을 계속 낸다고 가정 — 안전은 이 노드가 맡는다)
    토크에 NaN·inf 가 있거나 4 개가 아님 → DAMPING (set_command, 이전 토크를 이어 내지 않는다)
  응답 감시: reply_timeout 동안 응답이 없는 바퀴는 error = ERR_NO_REPLY (fault_monitor 가 0 이 아니면 세운다)
  모터 모드 다시 넣기: 응답이 없는 바퀴에는 reenter_period 마다 ENTER 를 다시 보낸다. 노드보다 모터가 늦게 켜지거나
    (시동 순서), 전압이 떨어져 모터가 다시 켜지면 모터 모드가 풀려 명령에 응답하지 않는다고 본다.
    2026-11-12 가짜 모터 시험에서 ENTER 를 한 번만 보냈더니 일부 모터만 켜져 로봇이 제자리에서 돌았다.
"""
import math

import numpy as np

from amr_mpc.ak40 import protocol as P

MODE_TORQUE, MODE_DAMPING = 0, 1          # WheelTorqueCommand.MODE_*
ERR_NO_REPLY = 0xFF                       # 모터 에러 코드와 겹치지 않게 255 를 "응답 없음" 으로 쓴다


class Ak40Bridge:
    def __init__(self, can_ids, direction, limits, damping_kd, tau_clamp, cmd_timeout, reply_timeout, temp_offset=40,
                 reenter_period=0.2, reply_can_id=None, reply_length=8):
        if len(can_ids) != 4 or len(direction) != 4 or len(set(can_ids)) != 4:
            raise ValueError(f"can_ids·direction 은 서로 다른 4개 [FL, FR, RL, RR] (받은 값 {can_ids}, {direction})")
        if any(d not in (1, -1) for d in direction):
            raise ValueError(f"direction 은 ±1 (받은 값 {direction})")
        self.ids = [int(i) for i in can_ids]
        self.index = {cid: i for i, cid in enumerate(self.ids)}
        self.dir = np.array(direction, float)
        self.lim = limits
        self.kd = float(damping_kd)
        self.tau_clamp = min(float(tau_clamp), limits.t_max)
        self.cmd_timeout, self.reply_timeout = float(cmd_timeout), float(reply_timeout)
        self.temp_offset = temp_offset
        self.reply_length = int(reply_length)
        if self.reply_length not in (6, 8):
            raise ValueError("reply_length must be 6 or 8")
        self.reply_can_id = reply_can_id        # None = 모터 자기 ID 로 온다 (매뉴얼), 숫자 = 그 ID
        self.unwrap = [P.PositionUnwrapper(limits) for _ in range(4)]
        self.pos = np.zeros(4)
        self.vel = np.zeros(4)
        self.tau = np.zeros(4)
        self.temp = np.full(4, math.nan)
        self.err = np.zeros(4, int)
        self.rx_t = np.full(4, -math.inf)       # 마지막 응답 시각
        self.fresh = np.zeros(4, bool)          # wheel_states() 이후 새 응답
        self.cycle_rx = np.zeros(4, bool)       # begin_cycle() 이후 받은 응답 — 이번 주기에 보낸 명령의 응답인가
        self.started_t = None
        self.reenter_period = float(reenter_period)
        self.enter_t = np.full(4, -math.inf)    # 마지막으로 ENTER 를 보낸 시각
        self.cmd_mode, self.cmd_tau, self.cmd_rx = None, np.zeros(4), -math.inf

    # ── 명령 ────────────────────────────────────────────
    def set_command(self, mode, torque, now):
        values = np.asarray(torque, float)
        if values.shape != (4,) or not np.isfinite(values).all():
            # 잘못된 명령으로 이전 토크가 계속 유지되지 않도록 감쇠로 전환한다.
            self.cmd_mode = MODE_DAMPING
            self.cmd_rx = float(now)
            return
        self.cmd_mode = int(mode)
        self.cmd_tau = np.asarray(torque, float).copy()
        self.cmd_rx = float(now)

    def start_frames(self, now):
        self.started_t = float(now)
        self.enter_t[:] = float(now)
        return [(cid, P.ENTER_MOTOR_MODE) for cid in self.ids]

    def reenter_frames(self, now):
        """응답이 reply_timeout 넘게 없는 바퀴에 ENTER 를 다시 (바퀴마다 reenter_period 에 한 번)."""
        out = []
        for i, cid in enumerate(self.ids):
            if now - self.rx_t[i] > self.reply_timeout and now - self.enter_t[i] >= self.reenter_period:
                self.enter_t[i] = float(now)
                out.append((cid, P.ENTER_MOTOR_MODE))
        return out

    def stop_frames(self):
        return [(cid, P.EXIT_MOTOR_MODE) for cid in self.ids]

    def active_mode(self, now):
        """이번 주기에 실제로 낼 모드 — 명령이 없거나 오래됐으면 DAMPING."""
        if self.cmd_mode == MODE_TORQUE and now - self.cmd_rx <= self.cmd_timeout:
            return MODE_TORQUE
        return MODE_DAMPING

    def command_frames(self, now, force_damping=False):
        """모터 4개의 MIT 명령 [(can_id, data)]. 반환 (frames, 낸 모드, 바퀴 기준 토크)."""
        mode = MODE_DAMPING if force_damping else self.active_mode(now)
        frames = []
        if mode == MODE_TORQUE:
            tau_w = np.clip(np.nan_to_num(self.cmd_tau), -self.tau_clamp, self.tau_clamp)
            for i, cid in enumerate(self.ids):
                frames.append((cid, P.pack_command(self.lim, 0.0, 0.0, 0.0, 0.0, self.dir[i] * tau_w[i], torque_limit=self.tau_clamp)))
        else:
            tau_w = np.zeros(4)
            for cid in self.ids:
                frames.append((cid, P.pack_command(self.lim, 0.0, 0.0, 0.0, self.kd, 0.0)))
        return frames, mode, tau_w

    # ── 응답 ────────────────────────────────────────────
    def is_reply(self, can_id, data):
        """이 프레임이 우리 모터의 응답인가. [0] 이 아는 모터 ID 이고 CAN ID 가 규칙에 맞아야 한다."""
        if len(data) != self.reply_length or data[0] not in self.index:
            return False
        return can_id == (data[0] if self.reply_can_id is None else int(self.reply_can_id))

    def on_reply(self, data, now):
        """응답 하나를 반영. 반환 바퀴 번호 (모르는 ID 면 None)."""
        r = P.unpack_reply(self.lim, data, self.temp_offset)
        i = self.index.get(r["id"])
        if i is None:
            return None
        d = self.dir[i]
        self.pos[i] = d * self.unwrap[i].update(r["p"])
        self.vel[i] = d * r["v"]
        self.tau[i] = d * r["t"]
        self.temp[i] = r["temp"]
        self.err[i] = r["error"]
        self.rx_t[i] = float(now)
        self.fresh[i] = True
        self.cycle_rx[i] = True
        return i

    def wheel_states(self, now):
        """WheelStates 에 넣을 값 dict. fresh 는 지난 호출 이후 응답이 왔는가. stamp = 가장 늦은 응답 시각."""
        err = self.err.copy()
        if self.started_t is not None and now - self.started_t > self.reply_timeout:
            late = (now - self.rx_t) > self.reply_timeout
            err[late] = ERR_NO_REPLY
        out = {"position": self.pos.copy(), "velocity": self.vel.copy(), "torque": self.tau.copy(),
               "temperature": self.temp.copy(), "error": err, "fresh": self.fresh.copy(),
               "stamp": float(np.max(self.rx_t)) if np.isfinite(self.rx_t).any() else float(now)}
        self.fresh[:] = False
        return out

    def begin_cycle(self):
        """명령을 보내기 직전에 부른다. 이후 on_reply 만 cycle_rx 를 채운다."""
        self.cycle_rx[:] = False

    def all_replied(self):
        """이번 주기에 보낸 명령에 네 모터가 모두 답했나.
        fresh 로 판단하면 안 된다 — 주기 시작에 반영한 지난 주기의 늦은 응답도 fresh 라서, 새 응답 2 개만 와도 4 개가 차
        기다리기를 일찍 끝내고, 나머지 새 응답은 또 다음 주기로 밀린다. 실차에서 RL·RR 이 늘 한 주기 늦게 쓰였다
        (2026-11-14, candump 로 응답은 1.2 ms 안에 왔는데 늦은 응답이 주기마다 2 개씩 쌓임)."""
        return bool(self.cycle_rx.all())
