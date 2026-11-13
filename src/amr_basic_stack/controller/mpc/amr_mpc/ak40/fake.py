#!/usr/bin/env python3
"""가짜 AK40 모터 4개 — 시뮬 플랜트가 CAN 으로만 대화하게 하는 부분 (sim_plant_torque_node can_interface:=vcan0).

진짜 모터가 하는 일만 흉내 낸다:
  - 특수 명령(들어가기/나오기/0점)과 MIT 명령을 받고, 받을 때마다 지금 상태로 응답한다
  - 모터 모드면 τ_모터 = kp(p_des − p) + kd(v_des − v) + t_ff 를 ±t_clamp 로 잘라 낸다. 모드 밖이면 0 (풀린 바퀴)
  - 명령이 끊겨도 마지막 명령을 계속 낸다 (명령 끊김 처리는 ak40_can_node 몫이라는 가정을 시험하려고)
  - 모터 모드 밖이면 MIT 명령에 응답하지 않는다 (가정 — 그래서 노드가 응답 없는 모터에 ENTER 를 다시 보낸다)
  - 장착 방향: direction[i] = +1 이면 모터 + = 바퀴 +. 위치는 ±p_max 에서 돈다. 숫자는 12/16 비트로 잘려 오간다
  - 응답 CAN ID 는 reply_can_id (None = 모터 자기 ID, 매뉴얼 61쪽) — reply_id(can_id) 로 얻는다
ROS·소켓에 의존하지 않는다.
"""
import numpy as np

from amr_mpc.ak40 import protocol as P


class FakeAk40Motors:
    def __init__(self, can_ids, direction, limits, t_clamp, temp_offset=40, reply_length=8, reply_can_id=None):
        self.ids = [int(i) for i in can_ids]
        self.index = {cid: i for i, cid in enumerate(self.ids)}
        self.dir = np.array(direction, float)
        self.lim = limits
        self.t_clamp = min(float(t_clamp), limits.t_max)
        self.temp_offset, self.reply_length = temp_offset, reply_length
        self.reply_can_id = reply_can_id
        self.enabled = np.zeros(4, bool)
        self.zero = np.zeros(4)                  # 0점 (모터 기준)
        self.cmd = np.zeros((4, 5))              # [p, v, kp, kd, t] (모터 기준)
        self.n_cmd = 0

    def reply_id(self, can_id):
        return can_id if self.reply_can_id is None else int(self.reply_can_id)

    def motor_state(self, wheel_pos, wheel_vel):
        p = self.dir * np.asarray(wheel_pos, float) - self.zero
        v = self.dir * np.asarray(wheel_vel, float)
        return p, v

    def torque(self, wheel_pos, wheel_vel):
        """지금 바퀴 상태에서 모터들이 내는 바퀴 기준 토크 (4,)."""
        p, v = self.motor_state(wheel_pos, wheel_vel)
        c = self.cmd
        tm = c[:, 2] * (c[:, 0] - p) + c[:, 3] * (c[:, 1] - v) + c[:, 4]
        tm = np.where(self.enabled, np.clip(tm, -self.t_clamp, self.t_clamp), 0.0)
        return self.dir * tm

    def on_frame(self, can_id, data, wheel_pos, wheel_vel, wheel_tau):
        """명령 프레임 하나 처리. 반환 응답 데이터 (응답 안 하면 None)."""
        i = self.index.get(can_id)
        if i is None or len(data) != 8:
            return None
        data = bytes(data)
        kind = P.SPECIAL.get(data)
        if kind == "enter":
            self.enabled[i] = True
            self.cmd[i] = 0.0
        elif kind == "exit":
            self.enabled[i] = False
        elif kind == "zero":
            self.zero[i] += self.motor_state(wheel_pos, wheel_vel)[0][i]
        else:
            if not self.enabled[i]:              # 모터 모드 밖에서는 MIT 명령을 무시하고 응답도 안 한다 (가정 — 매뉴얼로 확인)
                return None
            self.cmd[i] = P.unpack_command(self.lim, data)
            self.n_cmd += 1
        p, v = self.motor_state(wheel_pos, wheel_vel)
        t = self.dir[i] * float(wheel_tau[i])
        return P.pack_reply(self.lim, can_id, P.wrap_position(p[i], self.lim), v[i], t,
                            length=self.reply_length, temp_offset=self.temp_offset)
