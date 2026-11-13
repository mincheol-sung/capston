#!/usr/bin/env python3
# ak40_can_node — 바퀴 토크 명령 ↔ CubeMars AK40-10 × 4 (CAN, MIT 모드). 시뮬 플랜트 자리에 꽂는 실차 노드 (7단계).
#   ros2 run amr_mpc ak40_can_node                          # config/ak40.yaml 의 interface (can0)
#   ros2 run amr_mpc ak40_can_node --ros-args -p interface:=vcan0   # 가짜 모터 (시뮬 플랜트 can_interface:=vcan0)
# ---------------------------------------------------------------
#   /wheel_torque_cmd   (WheelTorqueCommand)  ← 게이트 출력. 시뮬 플랜트가 받던 것과 같다
#      ↓  rate_hz(100) 마다: 모터 4개에 MIT 명령 → 응답 4개를 reply_wait 안에 모아서 바로 발행
#   /wheel_states       (WheelStates)         → 추정기·torque_output·바퀴 PI·slip_logger·fault_monitor. 시뮬 플랜트와 같은 꼴
#                                              header.stamp = 가장 늦은 CAN 응답 수신 시각, fresh = 이번 주기에 응답이 왔나
#   /ak40_can/status    (Float64MultiArray)   [t, 낸 모드(0 토크/1 감쇠), 종단 지연 ms, 응답 수, 틱 ms, 누적 응답 누락]
#                                              종단 지연 = 송신 시각 − 명령 header.stamp(그 명령을 계산한 상태 추정값의 시각)
# 계산(명령 규칙·부호·감시)은 amr_mpc/ak40/bridge.py, CAN 메시지는 amr_mpc/ak40/protocol.py — 근거는 거기 머리주석.
#
# 안전
#   - config/ak40.yaml manual_checked: false 면 실제 버스에서 시작을 거부한다. vcan* 은 허용
#   - hardware_checked: false (모터 ID·부호 실물 미확인) 면 실제 버스에서 토크를 first_run_tau_limit(0.1 N·m)로 묶는다
#   - 시작: 모터 모드에 들어가 바로 DAMPING (게이트가 무장 전 DAMPING 을 내는 것과 같다).
#     응답이 없는 모터에는 0.2 s 마다 모터 모드 명령을 다시 보낸다 (늦게 켜진 모터·전압 강하로 꺼졌다 켜진 모터)
#   - 명령이 cmd_timeout 끊기면 DAMPING. 토크는 ±motor.tau_peak_Nm 로 자른다
#   - 응답이 reply_timeout 없으면 그 바퀴 error = 255 → fault_monitor 가 무장을 푼다
#   - 끌 때: DAMPING 을 shutdown_damping_s 동안 보낸 뒤 모터 모드에서 나온다 (나오면 바퀴가 풀린다)
# ---------------------------------------------------------------
import math
import signal
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float64MultiArray

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.ak40 import protocol as P
from amr_mpc.ak40.bridge import Ak40Bridge
from amr_mpc.ak40.bus import CanBus
from amr_mpc.core import config
from amr_mpc.torque.model import TorquePlant


def load_ak40():
    return config.load("ak40.yaml")["ak40"]


def reply_can_id(c):
    """config 의 reply_can_id: "motor" → None (모터 자기 ID 로 온다, 매뉴얼 61쪽), 숫자 → 그 ID."""
    v = c.get("reply_can_id", "motor")
    return None if str(v).strip().lower() == "motor" else int(v)


class Ak40CanNode(Node):
    def __init__(self):
        super().__init__("ak40_can_node")
        c = load_ak40()
        plant = TorquePlant.from_yaml()
        self.declare_parameters("", [
            ("interface", str(c["interface"])),
            ("allow_unverified_config", False),     # 시험용: manual_checked false 인 설정으로 실제 버스를 쓰겠다 (권하지 않음)
            ("cmd_topic", "/wheel_torque_cmd"),
            ("wheel_states_topic", "/wheel_states"),
        ])
        g = lambda n: self.get_parameter(n).value
        self.ifname = str(g("interface"))
        virtual = self.ifname.startswith("vcan")
        if not bool(c["manual_checked"]) and not virtual and not bool(g("allow_unverified_config")):
            raise RuntimeError(
                f"config/ak40.yaml 의 manual_checked 가 false 다 — MIT 범위·메시지 형식을 매뉴얼로 확인하기 전에는 "
                f"실제 버스({self.ifname})를 쓰지 않는다. 가짜 모터 시험은 interface:=vcan0")
        self.first_run = not bool(c["hardware_checked"]) and not virtual
        tau_clamp = float(c["first_run_tau_limit"]) if self.first_run else plant.tau_peak
        self.lim = P.MitLimits.from_dict(c["mit_limits"])
        self.bridge = Ak40Bridge(c["can_ids"], c["direction"], self.lim, plant.damping_kd, tau_clamp,
                                 float(c["cmd_timeout"]), float(c["reply_timeout"]), int(c["temp_offset"]),
                                 reply_can_id=reply_can_id(c))
        self.reply_wait = float(c["reply_wait"])
        self.shutdown_s = float(c["shutdown_damping_s"])
        self.period = 1.0 / float(c["rate_hz"])
        try:
            self.bus = CanBus(self.ifname)
        except OSError as e:
            raise RuntimeError(
                f"CAN 장치 {self.ifname} 를 열 수 없다 ({e}). 실차: sudo ip link set {self.ifname} up type can bitrate 1000000 "
                f"/ 가짜 모터: vcan0 을 먼저 만든다 (amr_mpc/ak40/bus.py 머리주석)") from None
        self.cmd_stamp = None
        self.missing = 0

        self.pub_ws = self.create_publisher(WheelStates, str(g("wheel_states_topic")), 10)
        self.pub_status = self.create_publisher(Float64MultiArray, "/ak40_can/status", 10)
        self.create_subscription(WheelTorqueCommand, str(g("cmd_topic")), self.on_cmd, 10)

        for cid, data in self.bridge.start_frames(self.now_s()):
            self.bus.send(cid, data)
        self.collect(0.05)
        self.create_timer(self.period, self.on_tick)
        lim = self.lim
        self.get_logger().info(
            f"ak40_can_node: {self.ifname}  모터 ID {self.bridge.ids} (FL FR RL RR)  부호 {list(map(int, self.bridge.dir))}\n"
            f"  MIT 범위 P ±{lim.p_max} V ±{lim.v_max} T ±{lim.t_max} (토크 한 칸 {lim.step('t')*1e3:.2f} mN·m)  "
            f"매뉴얼 {'확인' if c['manual_checked'] else '★ 미확인'}, 실물 {'확인' if c['hardware_checked'] else '★ 미확인'}\n"
            f"  {1/self.period:.0f} Hz, DAMPING kd {plant.damping_kd}, 토크 ±{self.bridge.tau_clamp} N·m, "
            f"명령 끊김 {self.bridge.cmd_timeout} s → DAMPING, 응답 없음 {self.bridge.reply_timeout} s → error 255")
        if self.first_run:
            self.get_logger().warning(
                f"hardware_checked: false — 모터 ID·장착 부호를 실물로 확인하기 전이라 토크를 ±{tau_clamp} N·m 로 묶었다. "
                f"바퀴를 띄우고 부호부터 확인할 것 (config/ak40.yaml direction 주석)")
        self.last_err = [0] * 4

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, m):
        self.bridge.set_command(m.mode, m.torque, self.now_s())
        self.cmd_stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def collect(self, wait):
        """응답을 모은다 — 4개가 다 오거나 wait 초가 지날 때까지."""
        t_end = time.monotonic() + wait
        while not self.bridge.all_replied():
            left = t_end - time.monotonic()
            if left <= 0.0:
                break
            f = self.bus.recv(left)
            if f is not None and self.bridge.is_reply(f[0], f[1]):
                self.bridge.on_reply(f[1], self.now_s())

    def send_all(self, frames):
        for cid, data in frames:
            try:
                self.bus.send(cid, data)
            except OSError as e:          # 버스 꺼짐·버퍼 넘침 — 응답 누락으로 드러나 fault_monitor 가 세운다
                self.get_logger().error(f"CAN 송신 실패 ({e})", throttle_duration_sec=1.0)

    def on_tick(self):
        t0 = time.monotonic()
        now = self.now_s()
        frames, mode, _ = self.bridge.command_frames(now)
        self.send_all(self.bridge.reenter_frames(now) + frames)
        self.collect(self.reply_wait)
        s = self.bridge.wheel_states(self.now_s())
        n_fresh = int(s["fresh"].sum())
        self.missing += 4 - n_fresh
        if n_fresh < 4:
            self.get_logger().warning(f"응답 {n_fresh}/4 — 빠진 바퀴 {[i for i in range(4) if not s['fresh'][i]]}",
                                      throttle_duration_sec=2.0)
        errs = [int(v) for v in s["error"]]
        if errs != self.last_err:
            for i, e in enumerate(errs):
                if e != self.last_err[i] and e != 0:
                    name = "응답 없음" if e == 255 else P.ERROR_NAMES.get(e, "알 수 없음")
                    self.get_logger().error(f"{['FL', 'FR', 'RL', 'RR'][i]} 모터 에러 {e} ({name})")
            self.last_err = errs
        ws = WheelStates()
        sec = int(s["stamp"])
        ws.header.stamp.sec, ws.header.stamp.nanosec = sec, int(round((s["stamp"] - sec) * 1e9))
        ws.header.frame_id = "base_footprint"
        ws.position = [float(v) for v in s["position"]]
        ws.velocity = [float(v) for v in s["velocity"]]
        ws.torque = [float(v) for v in s["torque"]]
        ws.temperature = [float(v) for v in s["temperature"]]
        ws.error = [int(v) for v in s["error"]]
        ws.fresh = [bool(v) for v in s["fresh"]]
        self.pub_ws.publish(ws)
        st = Float64MultiArray()
        lat = (now - self.cmd_stamp) * 1e3 if (self.cmd_stamp and mode == 0) else math.nan
        st.data = [now, float(mode), lat, float(n_fresh), (time.monotonic() - t0) * 1e3, float(self.missing)]
        self.pub_status.publish(st)

    def shutdown(self):
        """DAMPING 을 잠깐 보낸 뒤 모터 모드에서 나온다 (ROS 없이 소켓만 쓴다 — 종료 중에도 동작)."""
        t_end = time.monotonic() + self.shutdown_s
        while time.monotonic() < t_end:
            frames, _, _ = self.bridge.command_frames(self.now_s(), force_damping=True)
            self.send_all(frames)
            time.sleep(self.period)
        self.send_all(self.bridge.stop_frames())
        self.bus.close()


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = Ak40CanNode()
    except RuntimeError as e:
        print(f"[ak40_can_node] 시작 거부: {e}", flush=True)
        rclpy.shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.shutdown()
            print("[ak40_can_node] DAMPING 뒤 모터 모드 해제", flush=True)
        except Exception as e:
            print(f"[ak40_can_node] 종료 처리 실패: {e}", flush=True)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
