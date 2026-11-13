#!/usr/bin/env python3
# 토크 스택 시뮬 플랜트 — 실차 ak40_can_node 와 같은 토픽·타입으로 "가짜 로봇" 노릇을 한다.
# 나중에 실차로 갈 때는 이 노드만 ak40_can_node 로 바꾼다 (토크 MPC 전환 2단계).
#   ros2 run amr_mpc sim_plant_torque_node
# ---------------------------------------------------------------
#   /wheel_torque_cmd   (amr_interfaces/WheelTorqueCommand)  ← 게이트 출력. 실차에서는 ak40_can_node 가 받는다
#      ↓  1판 모델 Ẍ = M⁻¹(R(θ)(T⁻¹)ᵀτ + w)  (amr_mpc/torque/model.py, config/torque_plant.yaml)
#   /wheel_states       (amr_interfaces/WheelStates, 100 Hz)  ← 실차와 같은 토픽. 무슬립 바퀴속도
#   /sim/ground_truth   (nav_msgs/Odometry, 100 Hz)           ← 시뮬 전용 참값. pose = map, twist = 몸체
#   TF map → base_footprint                                   ← RViz 용 (publish_tf)
#
# 센서 흉내 (sensors:=true, 4단계 state_estimator 용 — amr_mpc/torque/sim_sensors.py)
#   /imu/base_link         (sensor_msgs/Imu, 200 Hz)        몸체 좌표 가속도·자이로. 바이어스 + 움직일 때 커지는 진동 잡음
#   /tracked_pose          (geometry_msgs/PoseStamped)      tracking_frame(카메라 IMU 위치)의 map 포즈. 스캔 매칭 10 Hz, 0.13 s 늦게
#   /scan_matched_points2  (sensor_msgs/PointCloud2, 빈 것)  slam_mode:=extrapolated 일 때 새 스캔 매칭이 반영되는 틱마다
#   센서를 켜면 물리 틱이 IMU 주기(200 Hz)가 되고, /wheel_states·참값은 그 절반(100 Hz)으로 낸다.
#
# 명령 처리 (실차 ak40_can_node 도 같은 규칙으로 만든다)
#   MODE_TORQUE   토크를 그대로 낸다. 피크 토크(motor.tau_peak_Nm)로 자른다.
#   MODE_DAMPING  τ = −kd·ω (MIT kp = 0, v = 0, kd = motor.damping_kd). 토크 0 은 관성 주행이라 정지가 아니다.
#   명령 없음 / cmd_timeout 초과 → DAMPING 과 같게 처리한다 (안전 쪽).
#
# 슬립 (surface_mu > 0, 5단계): 바퀴 각속도를 따로 상태로 두고 바닥 마찰력을 μ N 으로 자른다 (model.py SlipPlant).
#   /wheel_states.velocity 는 실제로 도는 바퀴 속도 (미끄러지면 몸체 속도와 안 맞는다). surface_mu ≤ 0 이면 예전 무슬립 모델.
# 구동계 마찰 (R2 리뷰 2026-11-12): 바퀴에 전달되는 토크 = τ − τ_fric(ω), τ_fric = coulomb·tanh(ω/ω_s) + viscous·ω.
#   값은 torque_plant.yaml friction (팀원 벤치 측정) × friction_scale. torque_output 이 같은 값으로 미리 더해 지운다.
#   friction_scale 0 이면 예전처럼 마찰 없음, 1.3 이면 "실제 마찰이 보상보다 30 % 큼" (보상 오차 시험).
#   ω_s(friction_smooth) 0.05 rad/s 는 보상 쪽 0.5 보다 작다 — 정지 근처에서는 보상이 실제와 어긋난다 (실차처럼).
# 아직 없는 것: SLAM 전역 최적화의 포즈 튐, 정지 마찰(stiction), 바퀴별 마찰 차이.
#
# 가짜 모터 모드 (can_interface:=vcan0, 7단계 2026-11-12) — ak40_can_node 까지 넣어 CAN 전 구간을 시험한다
#   /wheel_torque_cmd 를 받지 않고 /wheel_states 도 내지 않는다. 대신 CAN 버스에서 AK40 4개처럼 MIT 명령을 받아
#   τ = kp(p_des − p) + kd(v_des − v) + t_ff 를 내고, 명령마다 바로 응답한다 (amr_mpc/ak40/fake.py, config/ak40.yaml).
#   명령·응답 숫자는 12/16 비트로 잘려 오간다 — 실차와 같은 양자화. 응답은 수신 스레드가 곧바로 낸다 (진짜 모터처럼 1 ms 안).
# ---------------------------------------------------------------
import math
import signal
import threading

import numpy as np
import rclpy
import rclpy.time
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from tf2_ros import TransformBroadcaster

from amr_interfaces.msg import WheelStates, WheelTorqueCommand
from amr_mpc.torque.model import DrivetrainFriction, SlipPlant, TorquePlant
from amr_mpc.ak40.bus import CanBus
from amr_mpc.ak40.fake import FakeAk40Motors
from amr_mpc.core import config
from amr_mpc.torque.sim_sensors import DEFAULTS as SENSOR_DEFAULTS, SimSensors


class SimPlantTorqueNode(Node):
    def __init__(self):
        super().__init__("sim_plant_torque_node")
        base = TorquePlant.from_yaml()
        self.declare_parameters("", [
            ("cmd_topic", "/wheel_torque_cmd"),
            ("wheel_states_topic", "/wheel_states"),
            ("truth_topic", "/sim/ground_truth"),
            ("map_frame", "map"),
            ("base_frame", "base_footprint"),
            ("publish_tf", True),
            ("rate_hz", 100.0),          # 발행 주기 = 제어 주기 (1판 Ts 10 ms)
            ("substeps", 10),            # 발행 1 주기 안 적분 횟수 (DAMPING 은 ω 에 따라 토크가 바뀌므로 잘게)
            ("cmd_timeout", 0.2),        # 게이트 command_timeout 과 같게
            ("x0", 0.0), ("y0", 0.0), ("yaw0", 0.0),
            # 아래는 비우면 torque_plant.yaml 값. 적재 조건 바꿀 때 launch 인자로 덮어쓴다.
            ("payload_kg", -1.0),
            ("use_wheel_inertia", base.use_wheel_inertia),
            # 센서 흉내
            ("sensors", True),
            ("imu_rate_hz", 200.0),
            ("imu_topic", "/imu/base_link"),
            ("pose_topic", "/tracked_pose"),
            ("scan_matched_topic", "/scan_matched_points2"),
            ("slam_mode", SENSOR_DEFAULTS["slam_mode"]),          # scan_time | extrapolated
            ("slam_latency", SENSOR_DEFAULTS["slam_latency"]),    # [s] 스캔 시각 → 결과가 나오는 시각
            ("sensor_noise_scale", 1.0),                          # 0 이면 잡음 없음 (바이어스는 그대로)
            ("sensor_seed", 0),
            ("surface_mu", -1.0),        # 바닥 마찰계수. ≤ 0 이면 슬립 없음 (기존 모델)
            ("slip_v_eps", 0.02),        # [m/s] 마찰력을 부드럽게 잇는 미끄럼 속도
            ("friction_scale", 1.0),     # 구동계 마찰 = torque_plant.yaml friction × 이 값. 0 이면 마찰 없음
            ("friction_smooth", 0.05),   # [rad/s] 쿨롱 마찰을 잇는 바퀴 속도 (작을수록 진짜 쿨롱에 가깝고 적분이 잘아진다)
            ("can_interface", ""),       # 비우면 ROS 토픽 모드. vcan0 등을 주면 가짜 AK40 모터 모드 (머리주석)
        ])
        g = lambda n: self.get_parameter(n).value
        override = {"use_wheel_inertia": bool(g("use_wheel_inertia"))}
        if float(g("payload_kg")) >= 0.0:
            override["mass"] = float(config.section("body", "torque_plant.yaml")["mass_kg"]) + float(g("payload_kg"))
        self.p = TorquePlant.from_yaml(**override)

        self.X = np.array([g("x0"), g("y0"), g("yaw0")], float)
        fs = float(g("friction_scale"))
        self.fric = DrivetrainFriction(fs * self.p.fric_coulomb, fs * self.p.fric_viscous, float(g("friction_smooth")))
        if not self.fric.active():
            self.fric = None
        self.slip = (SlipPlant(self.p, float(g("surface_mu")), float(g("slip_v_eps")), fric=self.fric)
                     if float(g("surface_mu")) > 0 else None)
        self.w = np.zeros(4)                                      # 슬립 모델의 바퀴 각속도
        self.Xd = np.zeros(3)
        self.wheel_pos = np.zeros(4)
        self.cmd_mode = WheelTorqueCommand.MODE_DAMPING
        self.cmd_tau = np.zeros(4)
        self.last_cmd_t = None
        self.applied = np.zeros(4)
        self.t_last = None

        self.dt = 1.0 / float(g("rate_hz"))
        self.nsub = max(1, int(g("substeps")))
        # 센서: IMU 위치는 추정기 설정과 같은 값을 쓴다 (config/torque_estimator.yaml)
        self.sensors = None
        self.pub_div, self.tick_n = 1, 0
        if bool(g("sensors")):
            ecfg = config.section("estimator", "torque_estimator.yaml")
            k = float(g("sensor_noise_scale"))
            scfg = {"imu_offset_x": float(ecfg["imu_offset_x"]), "imu_offset_y": float(ecfg["imu_offset_y"]),
                    "slam_mode": str(g("slam_mode")), "slam_latency": float(g("slam_latency"))}
            for key in ("accel_noise_still", "accel_noise_moving", "gyro_noise_still", "gyro_noise_moving",
                        "slam_noise_xy", "slam_noise_yaw_deg"):
                scfg[key] = SENSOR_DEFAULTS[key] * k
            self.sensors = SimSensors(scfg, seed=int(g("sensor_seed")))
            self.pub_div = max(1, int(round(float(g("imu_rate_hz")) / float(g("rate_hz")))))
            self.dt = self.dt / self.pub_div                  # 물리 틱 = IMU 주기
            self.nsub = max(1, self.nsub // self.pub_div)
            self.pub_imu = self.create_publisher(Imu, str(g("imu_topic")), 20)
            self.pub_pose = self.create_publisher(PoseStamped, str(g("pose_topic")), 20)
            self.pub_smp = self.create_publisher(PointCloud2, str(g("scan_matched_topic")), 5)
        self.timeout = float(g("cmd_timeout"))
        self.map_frame, self.base_frame = str(g("map_frame")), str(g("base_frame"))

        self.fake, self.can, self.can_lock = None, None, threading.Lock()
        if str(g("can_interface")):
            ak = config.load("ak40.yaml")["ak40"]
            rid = str(ak.get("reply_can_id", "motor")).strip().lower()
            self.fake = FakeAk40Motors(ak["can_ids"], ak["direction"], AKP.MitLimits.from_dict(ak["mit_limits"]),
                                       self.p.tau_peak, int(ak["temp_offset"]), int(ak["reply_length"]),
                                       reply_can_id=None if rid == "motor" else int(rid))
            self.can = CanBus(str(g("can_interface")))
            threading.Thread(target=self.can_loop, daemon=True).start()
        self.pub_ws = self.create_publisher(WheelStates, str(g("wheel_states_topic")), 10)
        self.pub_truth = self.create_publisher(Odometry, str(g("truth_topic")), 10)
        self.tf = TransformBroadcaster(self) if bool(g("publish_tf")) else None
        self.create_subscription(WheelTorqueCommand, str(g("cmd_topic")), self.on_cmd, 10)
        self.create_timer(self.dt, self.on_tick)
        self.get_logger().info(
            f"시뮬 플랜트: m={self.p.mass:.2f} kg  I_z={self.p.inertia_z:.3f}  "
            f"바퀴관성 {'켬' if self.p.use_wheel_inertia else '끔'} (μ={self.p.mu:.3f} kg)  "
            f"kd={self.p.damping_kd}  피크 {self.p.tau_peak} N·m  물리 틱 {1/self.dt:.0f} Hz  "
            + ("슬립 없음" if self.slip is None else
               f"슬립 모델 μ {self.slip.mu:.2f} (바퀴당 최대 {self.slip.Fmax:.2f} N, 토크로 {self.p.r/math.sqrt(2)*self.slip.Fmax:.3f} N·m)")
            + ("  구동계 마찰 없음" if self.fric is None else
               f"  구동계 마찰 coulomb {self.fric.coulomb:.4f} N·m + viscous {self.fric.viscous:.5f}·ω (ω_s {self.fric.smooth} rad/s)")
            + ("" if self.sensors is None else
               f"\n  센서 흉내: IMU {1/self.dt:.0f} Hz @ ({self.sensors.r[0]:.3f}, {self.sensors.r[1]:.3f}) m, "
               f"SLAM {self.sensors.c['slam_mode']} 지연 {self.sensors.c['slam_latency']*1e3:.0f} ms, "
               f"잡음 배율 {float(g('sensor_noise_scale')):g}")
            + ("" if self.fake is None else
               f"\n  가짜 AK40 모터 모드: {g('can_interface')}  ID {self.fake.ids}  부호 {list(map(int, self.fake.dir))}"
               f"  (/wheel_torque_cmd·/wheel_states 대신 CAN)"))

    def can_loop(self):
        """가짜 모터 수신 스레드 — 명령을 받는 즉시 지금 상태로 응답한다."""
        while rclpy.ok():
            try:
                f = self.can.recv(0.1)
            except OSError:
                return
            if f is None:
                continue
            with self.can_lock:
                reply = self.fake.on_frame(f[0], f[1], self.wheel_pos, self._wheel_speeds(), self.applied)
            if reply is not None:
                try:
                    self.can.send(self.fake.reply_id(f[0]), reply)
                except OSError:
                    pass

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.cmd_mode = int(msg.mode)
        self.cmd_tau = np.clip(np.array(msg.torque, float), -self.p.tau_peak, self.p.tau_peak)
        self.last_cmd_t = self.now_s()

    def _wheel_speeds(self):
        """실제로 도는 바퀴 각속도 (슬립 모델이면 바퀴 상태, 아니면 몸체에서 역기구학)."""
        return self.w.copy() if self.slip is not None else self.p.wheel_speeds(self.X, self.Xd)

    def _torque(self, damping):
        if self.fake is not None:                    # 가짜 모터: 모터가 MIT 법칙으로 낸다 (끊김 처리는 ak40_can_node 몫)
            return self.fake.torque(self.wheel_pos, self._wheel_speeds())
        if damping:
            return np.clip(-self.p.damping_kd * self._wheel_speeds(), -self.p.tau_peak, self.p.tau_peak)
        return self.cmd_tau

    def on_tick(self):
        # 실제로 흐른 시간만큼 적분한다. 고정 10 ms 로 적분하고 시계 시각을 찍으면, 타이머가 1 ms 흔들릴 때
        # 상태와 stamp 가 1 ms 어긋나 1 m/s 에서 위치가 1 mm 튄 것처럼 보인다 (2026-11-11 폐루프 시험에서 확인).
        now = self.now_s()
        elapsed = self.dt if self.t_last is None else min(max(now - self.t_last, 0.0), 5.0 * self.dt)
        self.t_last = now
        fresh = self.last_cmd_t is not None and now - self.last_cmd_t <= self.timeout
        damping = (not fresh) or self.cmd_mode != WheelTorqueCommand.MODE_TORQUE
        self.can_lock.acquire()                      # 가짜 모터 명령·상태를 수신 스레드와 나눠 쓴다
        n = self.nsub
        if self.slip is not None:                    # 바퀴–바닥 마찰은 빠른 동역학이라 잘게 나눈다
            n = max(n, int(math.ceil(elapsed / self.slip.stable_dt())))
        elif self.fric is not None:                  # 구동계 마찰도 ω = 0 근처에서 뻣뻣하다
            n = max(n, int(math.ceil(elapsed / self.fric.stable_dt(self.p))))
        h = elapsed / n
        for _ in range(n):
            tau = self._torque(damping)
            w_before = self._wheel_speeds()
            if self.slip is not None:
                self.X, self.Xd, self.w = self.slip.step(self.X, self.Xd, self.w, tau, h)
            else:
                self.X, self.Xd = self.p.step(self.X, self.Xd, tau, h, fric=self.fric)
            self.wheel_pos += 0.5 * h * (w_before + self._wheel_speeds())
        self.applied = tau
        self.can_lock.release()
        if self.sensors is not None:
            self.publish_sensors(now)
        self.tick_n += 1
        if self.tick_n % self.pub_div == 0:
            self.publish(self._stamp(now))

    def _stamp(self, t):
        return rclpy.time.Time(nanoseconds=int(round(t * 1e9)), clock_type=self.get_clock().clock_type).to_msg()

    def publish_sensors(self, now):
        s = self.sensors.step(now, self.X, self.Xd)
        if s["imu"] is not None:
            m = Imu()
            m.header.stamp = self._stamp(now)
            m.header.frame_id = self.base_frame
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, s["imu"][:3])
            m.angular_velocity.z = float(s["imu"][3])
            m.orientation_covariance[0] = -1.0          # 자세는 주지 않는다 (REP 145)
            self.pub_imu.publish(m)
        for ts in s["scan_matched"]:                    # 포즈보다 먼저 낸다 (추정기가 "직후의 포즈"를 고른다)
            c = PointCloud2()
            c.header.stamp = self._stamp(ts)
            c.header.frame_id = self.map_frame
            self.pub_smp.publish(c)
        for (ts, x, y, yaw) in s["poses"]:
            ps = PoseStamped()
            ps.header.stamp = self._stamp(ts)
            ps.header.frame_id = self.map_frame
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.z, ps.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.pub_pose.publish(ps)

    def publish(self, stamp):
        wheel_w = self._wheel_speeds()

        if self.fake is None:                        # 가짜 모터 모드에서는 ak40_can_node 가 낸다
            self.publish_wheel_states(stamp, wheel_w)
        self.publish_truth(stamp)

    def publish_wheel_states(self, stamp, wheel_w):
        ws = WheelStates()
        ws.header.stamp = stamp
        ws.header.frame_id = self.base_frame
        ws.position = [float(v) for v in self.wheel_pos]
        ws.velocity = [float(v) for v in wheel_w]
        ws.torque = [float(v) for v in self.applied]
        ws.temperature = [float("nan")] * 4
        ws.error = [0, 0, 0, 0]
        ws.fresh = [True] * 4
        self.pub_ws.publish(ws)

    def publish_truth(self, stamp):
        qz, qw = math.sin(self.X[2] / 2), math.cos(self.X[2] / 2)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.map_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x, o.pose.pose.position.y = float(self.X[0]), float(self.X[1])
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        vb = self.p.body_velocity(self.X, self.Xd)
        o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.angular.z = map(float, vb)
        self.pub_truth.publish(o)

        if self.tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.map_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y = float(self.X[0]), float(self.X[1])
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf.sendTransform(t)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SimPlantTorqueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
