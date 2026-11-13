from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import math
import threading
import time
from typing import Optional, Tuple

from basicmicro import Basicmicro
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32MultiArray
from tf2_ros import TransformBroadcaster


@dataclass
class WheelSpeeds:
    front_left: float = 0.0
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0


@dataclass
class WheelQpps:
    front_left: int = 0
    front_right: int = 0
    rear_left: int = 0
    rear_right: int = 0


def gate_wheel_sample(
    value: float,
    last: Optional[float],
    dt: Optional[float],
    max_abs: float,
    max_accel: float,
    streak: int,
    max_streak: int,
) -> Tuple[bool, Optional[str], int]:
    """바퀴 속도 샘플 하나를 받을지 판정한다. (accept, 거부 사유, 새 연속거부 수).

    크기 게이트: |value| > max_abs 는 물리적으로 불가능하므로 항상 버린다.
    변화율 게이트: 마지막으로 받은 값 대비 |Δω| > max_accel·dt 면 버린다. 단
    max_streak 번 연속 버린 뒤에는 받아들인다 — 지속되는 변화는 진짜로 보고,
    문턱을 잘못 잡았을 때 마지막 값에 영원히 갇히는 것을 막는다.
    last/dt 가 None(아직 받은 값 없음)이면 변화율 게이트를 건너뛴다.
    """
    if not math.isfinite(value) or abs(value) > max_abs:
        return False, 'magnitude', streak
    if last is not None and dt is not None and abs(value - last) > max_accel * dt:
        if streak < max_streak:
            return False, 'step', streak + 1
    return True, None, 0


class RoboClaw:
    """Compatibility adapter around Basicmicro's official Python API."""

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout: float,
        mock: bool,
        require_ack: bool,
        logger,
        retries: int = 2,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.mock = mock
        self.require_ack = require_ack
        self.logger = logger
        self._lock = threading.Lock()
        self._driver = None
        self._last_io_warning = 0.0
        self._enc_prev = None          # encoder 모드: 직전 (시각, M1 카운트, M2 카운트)

        if self.mock:
            self.logger.info(f'RoboClaw mock mode enabled for {self.port}')
            return

        # retries: 실패한 읽기 하나가 timeout × retries 를 쓴다 (Basicmicro._read). 기본 2 는 라이브러리 기본값.
        self._driver = Basicmicro(
            self.port,
            self.baudrate,
            timeout=self.timeout,
            retries=retries,
        )
        if not self._driver.Open():
            self._driver = None
            raise RuntimeError(
                f'Failed to open RoboClaw serial port {self.port} '
                f'at {self.baudrate} baud'
            )
        if not self.require_ack:
            self.logger.warning(
                'require_ack=false is not supported by the official Basicmicro '
                'API; write acknowledgements remain enabled.'
            )
        self.logger.info(
            f'Opened RoboClaw serial port {self.port} at {self.baudrate} baud '
            'using the official Basicmicro library'
        )

    def close(self):
        with self._lock:
            if self._driver is not None:
                self._driver.close()
                self._driver = None

    def set_motor_speeds(
        self,
        address: int,
        m1_qpps: int,
        m2_qpps: int,
    ):
        if self.mock:
            self.logger.debug(
                f'[mock {self.port}] address={address} '
                f'M1={m1_qpps} qpps M2={m2_qpps} qpps'
            )
            return True

        with self._lock:
            try:
                success = self._driver.SpeedM1M2(
                    address,
                    int(m1_qpps),
                    int(m2_qpps),
                )
            except Exception as exc:
                self._warn_throttled(
                    f'RoboClaw speed write failed on {self.port} '
                    f'address={address}: {exc}'
                )
                return False
        if not success:
            self._warn_throttled(
                f'RoboClaw rejected speed command on {self.port} '
                f'address={address}.'
            )
        return bool(success)

    def read_main_battery_voltage(self, address: int) -> Optional[float]:
        if self.mock:
            return None

        with self._lock:
            try:
                result = self._driver.ReadMainBatteryVoltage(address)
            except Exception as exc:
                self._warn_throttled(
                    f'RoboClaw battery read failed on {self.port} '
                    f'address={address}: {exc}'
                )
                return None
        if not result or not result[0]:
            self._warn_throttled(
                f'RoboClaw battery read failed on {self.port} '
                f'address={address}.'
            )
            return None

        return float(result[1]) / 10.0

    def read_motor_speeds(
        self,
        address: int,
    ) -> Tuple[Optional[int], Optional[int]]:
        """Return measured M1/M2 encoder speeds in signed QPPS.

        A failed channel is returned as None so callers can retain the previous
        measurement instead of injecting a physically false zero.
        """
        if self.mock:
            return None, None

        with self._lock:
            m1_result = self._read_speed_result(
                self._driver.ReadSpeedM1,
                address,
                'M1',
            )
            m2_result = self._read_speed_result(
                self._driver.ReadSpeedM2,
                address,
                'M2',
            )
        return m1_result, m2_result

    def read_motor_speeds_combined(
        self,
        address: int,
    ) -> Tuple[Optional[int], Optional[int]]:
        """GetSpeeds (cmd 108) 한 번으로 M1/M2 를 읽는다. read_motor_speeds 의 트랜잭션 2 → 1.

        펌웨어가 108 을 지원해야 한다 — roboclaw_probe bench 로 먼저 확인할 것. 실패하면 두 채널 모두 None.
        두 값 모두 signed 32-bit (two's complement) 로 복원한다. status 바이트는 없다.
        """
        if self.mock:
            return None, None

        with self._lock:
            try:
                result = self._driver.GetSpeeds(address)
            except Exception as exc:
                self._warn_throttled(
                    f'RoboClaw GetSpeeds failed on {self.port} address={address}: {exc}'
                )
                return None, None
        if not result or not result[0]:
            self._warn_throttled(f'RoboClaw GetSpeeds failed on {self.port} address={address}.')
            return None, None
        return self._signed32(result[1]), self._signed32(result[2])

    def read_motor_speeds_encoder(
        self,
        address: int,
    ) -> Tuple[Optional[int], Optional[int]]:
        """엔코더 카운트(GetEncoders, cmd 78) 차분 / host 시각 차 → M1/M2 속도 [counts/s].

        2026-10-22 실측 (바퀴 띄움, roboclaw_probe delay): cmd 18 속도(ReadSpeedM1/M2)는 RoboClaw 펌웨어의
        속도 계산 필터 때문에 실제 바퀴보다 θ+τ 로 91~94 ms 늦다 (실제 θ 11 / τ 16 ms, cmd 18 θ 30 / τ 88 ms).
        카운트는 평균 창이 없다. 대가는 잡음 — 10 ms 차분 1σ 0.22~0.33 rad/s (cmd 18 은 0.017).
        시각은 트랜잭션 왕복의 가운데. 읽기 실패 뒤에는 공백 전체의 평균 속도가 나온다.
        첫 샘플은 짝이 없어 (None, None) — 호출자는 읽기 실패와 같이 다룬다.
        """
        if self.mock:
            return None, None

        with self._lock:
            t0 = time.monotonic()
            try:
                result = self._driver.GetEncoders(address)
            except Exception as exc:
                self._warn_throttled(
                    f'RoboClaw GetEncoders failed on {self.port} address={address}: {exc}'
                )
                return None, None
            t = (t0 + time.monotonic()) / 2.0
        if not result or not result[0]:
            self._warn_throttled(f'RoboClaw GetEncoders failed on {self.port} address={address}.')
            return None, None
        c1, c2 = self._signed32(result[1]), self._signed32(result[2])
        prev, self._enc_prev = self._enc_prev, (t, c1, c2)
        if prev is None or t - prev[0] < 1e-4:
            return None, None
        dt = t - prev[0]
        return (int(round(self._wrap32(c1 - prev[1]) / dt)),
                int(round(self._wrap32(c2 - prev[2]) / dt)))

    @staticmethod
    def _wrap32(delta: int) -> int:
        """32-bit 카운터가 한 바퀴 돌아도 차분이 맞게 [-2^31, 2^31) 로 접는다."""
        return ((int(delta) + 0x80000000) & 0xFFFFFFFF) - 0x80000000

    @staticmethod
    def _signed32(raw: int) -> int:
        raw = int(raw) & 0xFFFFFFFF
        return raw - 0x100000000 if raw >= 0x80000000 else raw

    def read_m1_velocity_pid(self, address: int):
        """Read M1 velocity P/I/D and full-power QPPS without changing them."""
        return self._read_velocity_pid(
            self._driver.ReadM1VelocityPID if self._driver is not None else None,
            address,
            "M1",
        )

    def read_m2_velocity_pid(self, address: int):
        """Read M2 velocity P/I/D and full-power QPPS without changing them."""
        return self._read_velocity_pid(
            self._driver.ReadM2VelocityPID if self._driver is not None else None,
            address,
            "M2",
        )

    def _read_velocity_pid(self, read_function, address: int, channel: str):
        """Use Basicmicro's command 55/56 implementation and CRC handling.

        The wire response is four unsigned, big-endian 32-bit values in
        P, I, D, QPPS order followed by CRC16. P/I/D are 16.16 fixed point,
        so the official library divides them by 65536. QPPS is returned as
        the unscaled encoder-counts-per-second value configured in RoboClaw.
        """
        if self.mock or read_function is None:
            return None

        with self._lock:
            try:
                result = read_function(address)
            except Exception as exc:
                self._warn_throttled(
                    f'RoboClaw {channel} velocity PID read failed on '
                    f'{self.port} address={address}: {exc}'
                )
                return None
        if not result or not result[0]:
            self._warn_throttled(
                f'RoboClaw {channel} velocity PID read failed on '
                f'{self.port} address={address}.'
            )
            return None
        return {
            "p": float(result[1]),
            "i": float(result[2]),
            "d": float(result[3]),
            "qpps": int(result[4]),
        }

    def read_motor_diagnostics(self, address: int):
        """Read signed PWM percentage and motor current for M1/M2."""
        if self.mock:
            return None

        with self._lock:
            try:
                pwm_result = self._driver.ReadPWMs(address)
                current_result = self._driver.ReadCurrents(address)
            except Exception as exc:
                self._warn_throttled(
                    f'RoboClaw motor diagnostic read failed on {self.port} '
                    f'address={address}: {exc}'
                )
                return None
        if not pwm_result or not pwm_result[0] or not current_result or not current_result[0]:
            self._warn_throttled(
                f'RoboClaw motor diagnostic read failed on {self.port} '
                f'address={address}.'
            )
            return None

        # Command 48 returns signed +/-32767 PWM; divide by 327.67 for %.
        # Command 49 returns current in 10 mA units; divide by 100 for A.
        return {
            "m1_pwm_percent": float(pwm_result[1]) / 327.67,
            "m2_pwm_percent": float(pwm_result[2]) / 327.67,
            "m1_current_a": float(current_result[1]) / 100.0,
            "m2_current_a": float(current_result[2]) / 100.0,
        }

    def _read_speed_result(
        self,
        read_function,
        address: int,
        channel: str,
    ) -> Optional[int]:
        try:
            result = read_function(address)
        except Exception as exc:
            self._warn_throttled(
                f'RoboClaw {channel} speed read failed on {self.port} '
                f'address={address}: {exc}'
            )
            return None
        if not result or not result[0]:
            self._warn_throttled(
                f'RoboClaw {channel} speed read failed on {self.port} '
                f'address={address}.'
            )
            return None

        raw_speed = int(result[1])

        # Basicmicro 는 32-bit 속도 필드를 unsigned long 으로 돌려준다. 이 보드
        # 펌웨어는 signed(two's complement) 로 보내므로 부호만 복원한다.
        #
        # 뒤에 오는 status 바이트는 쓰지 않는다. 예전 코드는 "크기 + 방향 bit"
        # 펌웨어를 가정해 status & 0x01 이면 부호를 뒤집었는데, 이 보드에서 그
        # bit 는 회전 방향이 아니다 — 감속(제동) 중인 양수 바퀴에서만 켜진다.
        # velocity_logs 3 개(2026-08-21, 09-01) 실측:
        #   +→- 뒤집힘 108 건, 크기는 5.71 → 4.68 → 3.83 처럼 끊김 없이 감쇠
        #   -→+ 뒤집힘   8 건, 전부 위 뒤집힘 구간이 2~6 샘플 뒤 끝나는 지점
        #   음수 바퀴 감속 샘플 1402 개(양수 684 개의 2 배)에서 시작한 뒤집힘 0 건
        # 크기+bit 펌웨어라면 음수 쪽 제동에서도 뒤집힘이 나와야 한다. 음수 raw 는
        # 위 분기가 이미 처리하므로 bit 를 볼 이유가 없다.
        if raw_speed >= 0x80000000:
            return raw_speed - 0x100000000
        return raw_speed

    def _warn_throttled(self, message: str):
        now = time.monotonic()
        if now - self._last_io_warning < 1.0:
            return
        self._last_io_warning = now
        self.logger.warning(message)


class RoboClawMecanumNode(Node):
    def __init__(self):
        super().__init__("roboclaw_mecanum_node")

        self.declare_parameter("left_port", "/dev/roboclaw_left")
        self.declare_parameter("right_port", "/dev/roboclaw_right")
        self.declare_parameter("baudrate", 38400)
        self.declare_parameter("serial_timeout_s", 0.05)
        self.declare_parameter("require_ack", True)
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("left_address", 128)
        self.declare_parameter("right_address", 128)
        self.declare_parameter("wheel_radius_m", 0.0762)
        self.declare_parameter("wheel_base_m", 0.1897)
        self.declare_parameter("robot_width_m", 0.207)
        self.declare_parameter("qpps_per_rad_per_sec", 1000.0)
        self.declare_parameter("max_wheel_rad_per_sec", 6.0)
        self.declare_parameter("cmd_timeout_s", 0.5)
        self.declare_parameter('feedback_timeout_s', 0.25)
        self.declare_parameter("battery_publish_rate_hz", 1.0)
        self.declare_parameter("feedback_report_period_s", 5.0)
        # 엔코더 속도 샘플 게이트. 근거는 base.yaml 주석.
        self.declare_parameter("feedback_max_wheel_rad_per_sec", 60.0)
        self.declare_parameter("feedback_max_wheel_accel", 1250.0)   # rad/s^2
        self.declare_parameter("feedback_max_step_rejects", 2)
        self.declare_parameter("diagnostic_publish_rate_hz", 5.0)
        # 100 Hz 전환용 (2026-10-22). 기본값은 전부 그 전 동작과 같다. 근거·측정법은 base.yaml 주석.
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("speed_read_mode", "per_motor")   # per_motor | combined | encoder
        self.declare_parameter("parallel_boards", False)
        self.declare_parameter("serial_retries", 2)
        # encoder_wheel_speeds 토픽: speed_read_mode 와 무관하게 엔코더 카운트 차분 속도를 따로 낸다.
        # C++ 100 Hz MPC 가 쓴다 (1차 모델이 실제 바퀴 속도 기준). /measured_wheel_speeds·/odom 은 그대로.
        self.declare_parameter("publish_encoder_speeds", True)

        # ── /odom 공분산 ─────────────────────────────────────────────────
        # 채우지 않으면 전부 0 으로 나간다. 0 공분산은 "이 측정에는 오차가 없다"는
        # 선언이라, EKF 가 휠 오도메트리를 절대 신뢰하게 되어 IMU·SLAM 보정이
        # 먹히지 않는다(칼만 게인이 0 에 수렴).
        #
        # twist: 엔코더 자체는 정밀하지만 메카넘 롤러 슬립이 오차를 지배한다.
        #        횡방향(vy)이 종방향(vx)보다, 회전(vyaw)이 그보다 더 나쁘다.
        #        기본값은 sigma 로 각각 0.1 m/s, 0.2 m/s, 0.3 rad/s 수준.
        # pose : 추측항법이라 불확실성이 되돌릴 수 없이 누적된다. 이동량·회전량에
        #        비례해 키우고 상한에서 멈춘다.
        # 실측 튜닝: 바닥에 표시를 두고 왕복 주행 후 /odom 과 실제 위치 차이를
        #            재서 growth 값을 맞출 것.
        self.declare_parameter("odom_twist_cov_vx", 0.01)      # (m/s)^2
        self.declare_parameter("odom_twist_cov_vy", 0.04)      # (m/s)^2
        self.declare_parameter("odom_twist_cov_vyaw", 0.09)    # (rad/s)^2
        self.declare_parameter("odom_pose_cov_initial", 1e-3)
        self.declare_parameter("odom_pose_cov_growth_xy", 0.02)   # m^2 per m 이동
        self.declare_parameter("odom_pose_cov_growth_yaw", 0.05)  # rad^2 per rad 회전
        self.declare_parameter("odom_pose_cov_max", 25.0)
        # 2D 주행이라 안 쓰는 축(z, roll, pitch). 크게 넣어 EKF 가 무시하게 한다.
        self.declare_parameter("odom_cov_unused", 1e6)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("invert_front_left", 1.0)
        self.declare_parameter("invert_front_right", 1.0)
        self.declare_parameter("invert_rear_left", 1.0)
        self.declare_parameter("invert_rear_right", 1.0)
        self.declare_parameter("front_left_output", "left_m2")
        self.declare_parameter("front_right_output", "right_m1")
        self.declare_parameter("rear_left_output", "left_m1")
        self.declare_parameter("rear_right_output", "right_m2")

        self.left_address = int(self.get_parameter("left_address").value)
        self.right_address = int(self.get_parameter("right_address").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base = float(self.get_parameter("wheel_base_m").value)
        self.robot_width = float(self.get_parameter("robot_width_m").value)
        self.qpps_per_rad_per_sec = float(self.get_parameter("qpps_per_rad_per_sec").value)
        self.max_wheel_rad_per_sec = float(self.get_parameter("max_wheel_rad_per_sec").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.feedback_timeout_s = float(self.get_parameter('feedback_timeout_s').value)
        if not math.isfinite(self.feedback_timeout_s) or self.feedback_timeout_s <= 0:
            raise ValueError('feedback_timeout_s must be finite and positive')
        self.battery_publish_rate_hz = float(self.get_parameter("battery_publish_rate_hz").value)
        self.feedback_report_period_s = float(
            self.get_parameter("feedback_report_period_s").value
        )
        self.diagnostic_publish_rate_hz = float(
            self.get_parameter("diagnostic_publish_rate_hz").value
        )
        self.feedback_max_wheel = float(
            self.get_parameter("feedback_max_wheel_rad_per_sec").value
        )
        self.feedback_max_accel = float(self.get_parameter("feedback_max_wheel_accel").value)
        self.feedback_max_step_rejects = int(
            self.get_parameter("feedback_max_step_rejects").value
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.odom_twist_cov_vx = float(self.get_parameter("odom_twist_cov_vx").value)
        self.odom_twist_cov_vy = float(self.get_parameter("odom_twist_cov_vy").value)
        self.odom_twist_cov_vyaw = float(self.get_parameter("odom_twist_cov_vyaw").value)
        self.odom_pose_cov_initial = float(self.get_parameter("odom_pose_cov_initial").value)
        self.odom_pose_cov_growth_xy = float(self.get_parameter("odom_pose_cov_growth_xy").value)
        self.odom_pose_cov_growth_yaw = float(self.get_parameter("odom_pose_cov_growth_yaw").value)
        self.odom_pose_cov_max = float(self.get_parameter("odom_pose_cov_max").value)
        self.odom_cov_unused = float(self.get_parameter("odom_cov_unused").value)
        if self.qpps_per_rad_per_sec <= 0.0:
            raise ValueError("qpps_per_rad_per_sec must be greater than zero")
        if self.wheel_radius <= 0.0:
            raise ValueError("wheel_radius_m must be greater than zero")
        if self.wheel_base + self.robot_width <= 0.0:
            raise ValueError("wheel_base_m + robot_width_m must be greater than zero")
        if self.diagnostic_publish_rate_hz < 0.0:
            raise ValueError("diagnostic_publish_rate_hz must not be negative")
        self.update_rate_hz = float(self.get_parameter("update_rate_hz").value)
        if not math.isfinite(self.update_rate_hz) or not 1.0 <= self.update_rate_hz <= 500.0:
            raise ValueError("update_rate_hz must be within [1, 500]")
        self.speed_read_mode = str(self.get_parameter("speed_read_mode").value)
        if self.speed_read_mode not in ("per_motor", "combined", "encoder"):
            raise ValueError("speed_read_mode must be 'per_motor', 'combined' or 'encoder'")
        self.parallel_boards = bool(self.get_parameter("parallel_boards").value)
        serial_retries = int(self.get_parameter("serial_retries").value)
        self.publish_encoder_speeds = bool(self.get_parameter("publish_encoder_speeds").value)
        if serial_retries < 1:
            raise ValueError("serial_retries must be at least 1")

        self.invert = {
            "front_left": float(self.get_parameter("invert_front_left").value),
            "front_right": float(self.get_parameter("invert_front_right").value),
            "rear_left": float(self.get_parameter("invert_rear_left").value),
            "rear_right": float(self.get_parameter("invert_rear_right").value),
        }
        self.motor_outputs = {
            "front_left": str(self.get_parameter("front_left_output").value),
            "front_right": str(self.get_parameter("front_right_output").value),
            "rear_left": str(self.get_parameter("rear_left_output").value),
            "rear_right": str(self.get_parameter("rear_right_output").value),
        }

        mock = bool(self.get_parameter("mock_mode").value)
        baudrate = int(self.get_parameter("baudrate").value)
        timeout = float(self.get_parameter("serial_timeout_s").value)
        require_ack = bool(self.get_parameter("require_ack").value)
        self.left_driver = RoboClaw(
            str(self.get_parameter("left_port").value),
            baudrate,
            timeout,
            mock,
            require_ack,
            self.get_logger(),
            serial_retries,
        )
        self.right_driver = RoboClaw(
            str(self.get_parameter("right_port").value),
            baudrate,
            timeout,
            mock,
            require_ack,
            self.get_logger(),
            serial_retries,
        )
        # 좌우 보드는 포트·잠금이 따로라 동시에 읽고 쓸 수 있다. 같은 보드 안은 RoboClaw._lock 이 직렬화한다.
        self._board_pool = (
            ThreadPoolExecutor(max_workers=2, thread_name_prefix='roboclaw')
            if self.parallel_boards and not mock else None
        )
        self.last_read_ms = math.nan
        self.last_write_ms = math.nan

        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_monotonic = time.monotonic()
        self.stop_pending = False
        self.last_cmd = Twist()
        self.current_wheels = WheelSpeeds()
        self.measured_wheels = WheelSpeeds()
        self.command_qpps = WheelQpps()
        self.measured_qpps = WheelQpps()
        self.feedback_attempts = {name: 0 for name in self.current_wheels.__dict__}
        self.feedback_failures = {name: 0 for name in self.current_wheels.__dict__}
        self.feedback_rejects = {name: 0 for name in self.current_wheels.__dict__}
        self.feedback_reject_streak = {name: 0 for name in self.current_wheels.__dict__}
        self.feedback_last_accept_time = {name: None for name in self.current_wheels.__dict__}
        self.last_feedback_report = time.monotonic()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.pose_cov_xy = self.odom_pose_cov_initial
        self.pose_cov_yaw = self.odom_pose_cov_initial
        self.last_odom_time = self.get_clock().now()
        self.last_joint_time = self.get_clock().now()
        self.wheel_positions = WheelSpeeds()

        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
        # 구조 B: 바퀴별 각속도 명령 [FL, FR, RL, RR] rad/s. 역기구학(twist_to_wheels)을 거치지 않고
        # 한계·반전만 적용해 보낸다. cmd_vel 과 같은 워치독(cmd_timeout_s)을 쓴다.
        self.wheel_cmd_sub = self.create_subscription(
            Float32MultiArray, "wheel_cmd", self.wheel_cmd_callback, 10)
        self.wheel_pub = self.create_publisher(Float32MultiArray, "wheel_speeds", 10)
        # [FL, FR, RL, RR] rad/s, 엔코더 카운트 차분 (publish_encoder_wheel_speeds)
        self.encoder_wheel_pub = self.create_publisher(Float32MultiArray, "encoder_wheel_speeds", 10)
        self.measured_wheel_pub = self.create_publisher(
            Float32MultiArray,
            "measured_wheel_speeds",
            10,
        )
        self.command_qpps_pub = self.create_publisher(
            Int32MultiArray,
            "roboclaw/command_qpps",
            10,
        )
        self.measured_qpps_pub = self.create_publisher(
            Int32MultiArray,
            "roboclaw/measured_qpps",
            10,
        )
        pid_config_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pid_config_pub = self.create_publisher(
            Float64MultiArray,
            "roboclaw/pid_config",
            pid_config_qos,
        )
        self.motor_diagnostic_pub = self.create_publisher(
            Float32MultiArray,
            "roboclaw/motor_diagnostics",
            10,
        )
        # [이번 주기 속도 읽기 ms, 마지막 속도 쓰기 ms, 주기 Hz 설정값]. 100 Hz 가 들어가는지 bag 으로 확인하는 용도
        self.io_timing_pub = self.create_publisher(Float32MultiArray, "roboclaw/io_timing", 10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.roboclaw_battery_pub = self.create_publisher(
            Float32MultiArray,
            "roboclaw/battery_voltages",
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.timer = self.create_timer(1.0 / self.update_rate_hz, self.update)
        if self.battery_publish_rate_hz > 0.0:
            self.battery_timer = self.create_timer(
                1.0 / self.battery_publish_rate_hz,
                self.publish_battery_state,
            )
        if self.diagnostic_publish_rate_hz > 0.0:
            self.diagnostic_timer = self.create_timer(
                1.0 / self.diagnostic_publish_rate_hz,
                self.publish_motor_diagnostics,
            )
        self.get_logger().info(
            "Runtime drive parameters: "
            f"wheel_radius_m={self.wheel_radius:.6g}, "
            f"qpps_per_rad_per_sec={self.qpps_per_rad_per_sec:.6g}, "
            f"max_wheel_rad_per_sec={self.max_wheel_rad_per_sec:.6g}, "
            f"update_rate_hz={self.update_rate_hz:.6g}, speed_read_mode={self.speed_read_mode}, "
            f"parallel_boards={self.parallel_boards}, serial_retries={serial_retries}"
        )
        self.pid_configs = self.read_and_publish_pid_configs()
        self.get_logger().info(
            "Motor diagnostics topic order: "
            "[FL PWM %, FR PWM %, RL PWM %, RR PWM %, "
            "FL current A, FR current A, RL current A, RR current A]"
        )
        self.get_logger().info("RoboClaw mecanum node started with encoder-based feedback")

    def destroy_node(self):
        self.stop_all()
        if getattr(self, '_board_pool', None) is not None:
            self._board_pool.shutdown(wait=True)
        self.left_driver.close()
        self.right_driver.close()
        super().destroy_node()

    def cmd_vel_callback(self, msg: Twist):
        if self.stop_pending or not self.feedback_is_fresh():
            self.stop_all()
            return
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_monotonic = time.monotonic()
        self.current_wheels = self.twist_to_wheels(msg)
        if not self.send_wheel_speeds(self.current_wheels):
            self.stop_all()
        self.publish_wheel_speeds(self.current_wheels)

    def wheel_cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warning(f"wheel_cmd length {len(msg.data)} != 4, ignored")
            return
        if self.stop_pending or not self.feedback_is_fresh():
            self.stop_all()
            return
        self.last_cmd_time = self.get_clock().now()
        self.last_cmd_monotonic = time.monotonic()
        self.current_wheels = self.limit_and_apply_inversion(WheelSpeeds(*[float(v) for v in msg.data]))
        if not self.send_wheel_speeds(self.current_wheels):
            self.stop_all()
        self.publish_wheel_speeds(self.current_wheels)

    def feedback_is_fresh(self):
        """Require a recently accepted sample from every physical wheel."""
        if self.left_driver.mock and self.right_driver.mock:
            return True
        now = time.monotonic()
        return all(stamp is not None and 0 <= now - stamp <= self.feedback_timeout_s
                   for stamp in self.feedback_last_accept_time.values())

    def update(self):
        now = self.get_clock().now()
        elapsed_since_cmd = time.monotonic() - self.last_cmd_monotonic
        if self.stop_pending or elapsed_since_cmd > self.cmd_timeout_s:
            if self.stop_pending or any(
                    abs(v) > 1e-6 for v in self.current_wheels.__dict__.values()):
                self.stop_all()
                self.publish_wheel_speeds(self.current_wheels)

        self.read_encoder_feedback()
        if not self.feedback_is_fresh():
            self.stop_all()
            # Held measurements must not become new measurements or be integrated
            # across the gap when feedback recovers. Downstream timeouts now expire.
            self.last_odom_time = now
            self.last_joint_time = now
            self.report_feedback_health()
            return
        self.publish_measured_wheel_speeds()
        self.publish_measured_qpps()
        self.publish_joint_state(now)
        self.publish_measured_odometry(now)
        self.report_feedback_health()

    def _on_both_boards(self, left_call, right_call):
        """좌우 보드에 같은 일을 한다. parallel_boards 면 동시에, 아니면 왼쪽 → 오른쪽."""
        pool = getattr(self, '_board_pool', None)
        if pool is None:
            return left_call(), right_call()
        f_left, f_right = pool.submit(left_call), pool.submit(right_call)
        return f_left.result(), f_right.result()

    def read_encoder_feedback(self):
        if self.left_driver.mock and self.right_driver.mock:
            return

        t_read = time.monotonic()
        mode = getattr(self, 'speed_read_mode', 'per_motor')
        if mode == 'combined':
            (left_m1, left_m2), (right_m1, right_m2) = self._on_both_boards(
                lambda: self.left_driver.read_motor_speeds_combined(self.left_address),
                lambda: self.right_driver.read_motor_speeds_combined(self.right_address))
        elif mode == 'encoder':
            (left_m1, left_m2), (right_m1, right_m2) = self._on_both_boards(
                lambda: self.left_driver.read_motor_speeds_encoder(self.left_address),
                lambda: self.right_driver.read_motor_speeds_encoder(self.right_address))
        else:
            (left_m1, left_m2), (right_m1, right_m2) = self._on_both_boards(
                lambda: self.left_driver.read_motor_speeds(self.left_address),
                lambda: self.right_driver.read_motor_speeds(self.right_address))
        if mode == 'encoder':
            enc_qpps = (left_m1, left_m2, right_m1, right_m2)
        elif getattr(self, 'publish_encoder_speeds', False):
            (e_l1, e_l2), (e_r1, e_r2) = self._on_both_boards(
                lambda: self.left_driver.read_motor_speeds_encoder(self.left_address),
                lambda: self.right_driver.read_motor_speeds_encoder(self.right_address))
            enc_qpps = (e_l1, e_l2, e_r1, e_r2)
        else:
            enc_qpps = None
        if enc_qpps is not None:
            self.publish_encoder_wheel_speeds(dict(zip(("left_m1", "left_m2", "right_m1", "right_m2"), enc_qpps)))
        self.last_read_ms = (time.monotonic() - t_read) * 1e3
        if hasattr(self, 'io_timing_pub'):
            self.io_timing_pub.publish(Float32MultiArray(data=[
                float(self.last_read_ms), float(getattr(self, 'last_write_ms', math.nan)),
                float(getattr(self, 'update_rate_hz', 50.0))]))
        output_qpps = {
            "left_m1": left_m1,
            "left_m2": left_m2,
            "right_m1": right_m1,
            "right_m2": right_m2,
        }

        for wheel_name, output_name in self.motor_outputs.items():
            self.feedback_attempts[wheel_name] += 1
            qpps = output_qpps.get(output_name)
            if qpps is None:
                self.feedback_failures[wheel_name] += 1
                continue

            # The output command uses this factor, so applying it again converts
            # encoder/motor sign back to the logical wheel convention.
            logical_qpps = int(round(qpps * self.invert[wheel_name]))
            logical_rad_s = float(logical_qpps) / self.qpps_per_rad_per_sec

            # CRC 를 통과한 쓰레기 값을 거른다. 2026-09-23 bag 135606 t=314.24 s:
            # rear_right 한 샘플이 -16,779,142 qpps(≈ -2^24, -79,870 rad/s) 로 읽혀
            # /odom 이 22.6 m 순간이동했고 cartographer 측위도 21 m 날아갔다.
            # 거른 샘플은 읽기 실패와 같이 마지막 정상값을 유지한다.
            now_mono = time.monotonic()
            last_t = self.feedback_last_accept_time[wheel_name]
            accept, reason, self.feedback_reject_streak[wheel_name] = gate_wheel_sample(
                logical_rad_s,
                None if last_t is None else getattr(self.measured_wheels, wheel_name),
                None if last_t is None else now_mono - last_t,
                self.feedback_max_wheel,
                self.feedback_max_accel,
                self.feedback_reject_streak[wheel_name],
                self.feedback_max_step_rejects,
            )
            if not accept:
                self.feedback_rejects[wheel_name] += 1
                self.get_logger().warning(
                    f"Rejected {wheel_name} encoder sample ({reason}): "
                    f"{logical_rad_s:.1f} rad/s ({logical_qpps} qpps), "
                    f"holding {getattr(self.measured_wheels, wheel_name):.2f} rad/s",
                    throttle_duration_sec=1.0,
                )
                continue
            self.feedback_last_accept_time[wheel_name] = now_mono
            setattr(self.measured_qpps, wheel_name, logical_qpps)
            setattr(self.measured_wheels, wheel_name, logical_rad_s)

    def publish_encoder_wheel_speeds(self, output_qpps):
        """엔코더 차분 속도를 논리 바퀴 순서 [FL, FR, RL, RR] rad/s 로 낸다.

        한 바퀴라도 읽기 실패(첫 샘플 포함)거나 크기 게이트(feedback_max_wheel_rad_per_sec)를 넘으면 이번 주기는
        내지 않는다 — 받는 쪽(C++ MPC)이 '새 측정 없음'으로 다루고 관측기는 예측만 한다.
        변화율 게이트는 /measured_wheel_speeds 쪽에만 있다 (차분 잡음 1σ 0.35 rad/s 는 그 문턱보다 훨씬 작다).
        """
        values = []
        for wheel_name in ("front_left", "front_right", "rear_left", "rear_right"):
            qpps = output_qpps.get(self.motor_outputs[wheel_name])
            if qpps is None:
                return
            rad_s = float(int(round(qpps * self.invert[wheel_name]))) / self.qpps_per_rad_per_sec
            if not math.isfinite(rad_s) or abs(rad_s) > self.feedback_max_wheel:
                return
            values.append(rad_s)
        if hasattr(self, 'encoder_wheel_pub'):
            self.encoder_wheel_pub.publish(Float32MultiArray(data=values))

    def report_feedback_health(self):
        if self.feedback_report_period_s <= 0.0:
            return
        now = time.monotonic()
        if now - self.last_feedback_report < self.feedback_report_period_s:
            return
        self.last_feedback_report = now

        reports = []
        has_failure = False
        for name in self.feedback_attempts:
            attempts = self.feedback_attempts[name]
            failures = self.feedback_failures[name]
            if attempts == 0:
                continue
            rejects = self.feedback_rejects[name]
            has_failure = has_failure or failures > 0 or rejects > 0
            reports.append(
                f"{name}={100.0 * failures / attempts:.1f}% ({failures}/{attempts}"
                f", rejected {rejects})")
        if has_failure:
            self.get_logger().warning(
                "Encoder feedback read failure rate (holding last valid value): "
                + ", ".join(reports)
            )
# 역기구학 행렬 Kinematic Model of a Four Mecanum Wheeled Mobile  Robot 논문과 동일
    def twist_to_wheels(self, msg: Twist) -> WheelSpeeds:
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)
        k = (self.wheel_base + self.robot_width) / 2.0
        r = self.wheel_radius

        speeds = WheelSpeeds(
            front_left=(vx - vy - k * wz) / r,
            front_right=(vx + vy + k * wz) / r,
            rear_left=(vx + vy - k * wz) / r,
            rear_right=(vx - vy + k * wz) / r,
        )
        return self.limit_and_apply_inversion(speeds)

    def limit_and_apply_inversion(self, speeds: WheelSpeeds) -> WheelSpeeds:
        values = {
            "front_left": speeds.front_left * self.invert["front_left"],
            "front_right": speeds.front_right * self.invert["front_right"],
            "rear_left": speeds.rear_left * self.invert["rear_left"],
            "rear_right": speeds.rear_right * self.invert["rear_right"],
        }
        max_abs = max(abs(value) for value in values.values())
        if max_abs > self.max_wheel_rad_per_sec:
            scale = self.max_wheel_rad_per_sec / max_abs
            values = {key: value * scale for key, value in values.items()}

        return WheelSpeeds(**values)

    def wheel_rad_s_to_qpps(self, wheel_rad_s: float) -> int:
        return int(round(wheel_rad_s * self.qpps_per_rad_per_sec))

    def send_wheel_speeds(self, speeds: WheelSpeeds):
        outputs = {
            "left_m1": 0,
            "left_m2": 0,
            "right_m1": 0,
            "right_m2": 0,
        }
        wheel_speeds = {
            "front_left": speeds.front_left,
            "front_right": speeds.front_right,
            "rear_left": speeds.rear_left,
            "rear_right": speeds.rear_right,
        }
        command_qpps = {}

        for wheel_name, wheel_speed in wheel_speeds.items():
            output_name = self.motor_outputs[wheel_name]
            if output_name not in outputs:
                self.get_logger().error(
                    f"Invalid motor output '{output_name}' for {wheel_name}. "
                    "Use one of: left_m1, left_m2, right_m1, right_m2."
                )
                continue
            qpps = self.wheel_rad_s_to_qpps(wheel_speed)
            command_qpps[wheel_name] = qpps
            outputs[output_name] = qpps

        self.command_qpps = WheelQpps(**command_qpps)
        self.publish_command_qpps()

        t_write = time.monotonic()
        left_ok, right_ok = self._on_both_boards(
            lambda: self.left_driver.set_motor_speeds(
                self.left_address, outputs['left_m1'], outputs['left_m2']),
            lambda: self.right_driver.set_motor_speeds(
                self.right_address, outputs['right_m1'], outputs['right_m2']))
        self.last_write_ms = (time.monotonic() - t_write) * 1e3
        return bool(left_ok and right_ok)

    def stop_all(self):
        self.current_wheels = WheelSpeeds()
        self.last_cmd = Twist()
        self.stop_pending = True
        try:
            left_ok = self.left_driver.set_motor_speeds(self.left_address, 0, 0)
            right_ok = self.right_driver.set_motor_speeds(self.right_address, 0, 0)
            self.stop_pending = not (left_ok and right_ok)
        except Exception as exc:  # pragma: no cover - defensive shutdown path
            self.get_logger().error(f"Failed to stop RoboClaw drivers: {exc}")
        if self.stop_pending:
            self.get_logger().error(
                'RoboClaw stop unconfirmed; retrying on subsequent updates',
                throttle_duration_sec=1.0)
        self.command_qpps = WheelQpps()
        if hasattr(self, "command_qpps_pub") and rclpy.ok():
            self.publish_command_qpps()
        return not self.stop_pending

    def publish_wheel_speeds(self, speeds: WheelSpeeds):
        msg = Float32MultiArray()
        msg.data = [speeds.front_left, speeds.front_right, speeds.rear_left, speeds.rear_right]
        self.wheel_pub.publish(msg)

    def publish_measured_wheel_speeds(self):
        msg = Float32MultiArray()
        msg.data = self.wheel_values(self.measured_wheels)
        self.measured_wheel_pub.publish(msg)

    def publish_command_qpps(self):
        msg = Int32MultiArray()
        msg.data = self.wheel_values(self.command_qpps)
        self.command_qpps_pub.publish(msg)

    def publish_measured_qpps(self):
        msg = Int32MultiArray()
        msg.data = self.wheel_values(self.measured_qpps)
        self.measured_qpps_pub.publish(msg)

    def read_and_publish_pid_configs(self):
        """Read and report the four physical RoboClaw velocity PID configs."""
        configs = {
            "left_m1": self.left_driver.read_m1_velocity_pid(self.left_address),
            "left_m2": self.left_driver.read_m2_velocity_pid(self.left_address),
            "right_m1": self.right_driver.read_m1_velocity_pid(self.right_address),
            "right_m2": self.right_driver.read_m2_velocity_pid(self.right_address),
        }
        wheel_for_output = {
            output_name: wheel_name
            for wheel_name, output_name in self.motor_outputs.items()
        }
        lines = [
            "RoboClaw velocity PID config (read-only):",
            "  ROS qpps_per_rad_per_sec="
            f"{self.qpps_per_rad_per_sec:.6g} "
            "(wheel rad/s <-> encoder counts/s conversion)",
            "  RoboClaw configured QPPS is the encoder speed at 100% power; "
            "it is not the ROS conversion factor.",
        ]
        output_order = ("left_m1", "left_m2", "right_m1", "right_m2")
        topic_values = []
        for output_name in output_order:
            config = configs[output_name]
            wheel_name = wheel_for_output.get(output_name, "unmapped")
            if config is None:
                lines.append(f"  {output_name} ({wheel_name}): read unavailable")
                topic_values.extend([math.nan] * 4)
                continue
            lines.append(
                f"  {output_name} ({wheel_name}): "
                f"P={config['p']:.9g}, I={config['i']:.9g}, "
                f"D={config['d']:.9g}, QPPS={config['qpps']}"
            )
            topic_values.extend(
                [config["p"], config["i"], config["d"], config["qpps"]]
            )

        self.get_logger().info("\n".join(lines))
        msg = Float64MultiArray()
        # Physical output order, four values per output:
        # left_m1, left_m2, right_m1, right_m2; each [P, I, D, QPPS].
        msg.data = topic_values
        self.pid_config_pub.publish(msg)
        return configs

    def publish_motor_diagnostics(self):
        """Publish low-rate PWM/current data in logical wheel order."""
        controller_data = {
            "left": self.left_driver.read_motor_diagnostics(self.left_address),
            "right": self.right_driver.read_motor_diagnostics(self.right_address),
        }
        pwm_percent = []
        current_a = []
        for wheel_name in self.current_wheels.__dict__:
            output_name = self.motor_outputs[wheel_name]
            controller_name, motor_name = output_name.split("_", maxsplit=1)
            diagnostics = controller_data[controller_name]
            if diagnostics is None:
                pwm_percent.append(math.nan)
                current_a.append(math.nan)
                continue
            # Match the logical wheel sign used by measured wheel feedback.
            pwm_percent.append(
                diagnostics[f"{motor_name}_pwm_percent"] * self.invert[wheel_name]
            )
            current_a.append(diagnostics[f"{motor_name}_current_a"])

        msg = Float32MultiArray()
        msg.data = pwm_percent + current_a
        self.motor_diagnostic_pub.publish(msg)

    @staticmethod
    def wheel_values(wheels):
        """Return wheel values in the public FL, FR, RL, RR topic order."""
        return [
            wheels.front_left,
            wheels.front_right,
            wheels.rear_left,
            wheels.rear_right,
        ]

    def publish_battery_state(self):
        left_voltage = self.left_driver.read_main_battery_voltage(self.left_address)
        right_voltage = self.right_driver.read_main_battery_voltage(self.right_address)
        voltages = [voltage for voltage in [left_voltage, right_voltage] if voltage is not None]
        if not voltages:
            return

        voltage_msg = Float32MultiArray()
        voltage_msg.data = [
            float("nan") if left_voltage is None else left_voltage,
            float("nan") if right_voltage is None else right_voltage,
        ]
        self.roboclaw_battery_pub.publish(voltage_msg)

        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.header.frame_id = self.base_frame_id
        battery.voltage = min(voltages)
        battery.present = True
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        battery.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        battery.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        self.battery_pub.publish(battery)

    def publish_joint_state(self, now):
        dt = (now - self.last_joint_time).nanoseconds / 1e9
        self.last_joint_time = now
        if dt > 0.0:
            self.wheel_positions.front_left += self.measured_wheels.front_left * dt
            self.wheel_positions.front_right += self.measured_wheels.front_right * dt
            self.wheel_positions.rear_left += self.measured_wheels.rear_left * dt
            self.wheel_positions.rear_right += self.measured_wheels.rear_right * dt

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = ["front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"]
        msg.position = [
            self.wheel_positions.front_left,
            self.wheel_positions.front_right,
            self.wheel_positions.rear_left,
            self.wheel_positions.rear_right,
        ]
        msg.velocity = [
            self.measured_wheels.front_left,
            self.measured_wheels.front_right,
            self.measured_wheels.rear_left,
            self.measured_wheels.rear_right,
        ]
        self.joint_pub.publish(msg)
# 정기구학 행렬 Kinematic Model of a Four Mecanum Wheeled Mobile  Robot 논문과 동일
    def publish_measured_odometry(self, now):
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        if dt <= 0.0:
            return

        fl = self.measured_wheels.front_left
        fr = self.measured_wheels.front_right
        rl = self.measured_wheels.rear_left
        rr = self.measured_wheels.rear_right
        k = (self.wheel_base + self.robot_width) / 2.0
        vx = self.wheel_radius * (fl + fr + rl + rr) / 4.0
        vy = self.wheel_radius * (-fl + fr + rl - rr) / 4.0
        wz = self.wheel_radius * (-fl + fr - rl + rr) / (4.0 * k)
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (vx * cos_yaw - vy * sin_yaw) * dt
        self.y += (vx * sin_yaw + vy * cos_yaw) * dt
        self.yaw = self.normalize_angle(self.yaw + wz * dt)

        # 추측항법이라 위치 불확실성은 되돌릴 수 없이 누적된다. 실제로 움직인
        # 거리·각도에 비례해 키운다(정지 중에는 자라지 않는다). 상한에서 멈춘다.
        self.pose_cov_xy = min(
            self.pose_cov_xy + self.odom_pose_cov_growth_xy * math.hypot(vx, vy) * dt,
            self.odom_pose_cov_max)
        self.pose_cov_yaw = min(
            self.pose_cov_yaw + self.odom_pose_cov_growth_yaw * abs(wz) * dt,
            self.odom_pose_cov_max)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # 대각 공분산. 순서는 [x, y, z, roll, pitch, yaw].
        unused = self.odom_cov_unused
        odom.pose.covariance[0] = self.pose_cov_xy
        odom.pose.covariance[7] = self.pose_cov_xy
        odom.pose.covariance[14] = unused
        odom.pose.covariance[21] = unused
        odom.pose.covariance[28] = unused
        odom.pose.covariance[35] = self.pose_cov_yaw
        odom.twist.covariance[0] = self.odom_twist_cov_vx
        odom.twist.covariance[7] = self.odom_twist_cov_vy
        odom.twist.covariance[14] = unused
        odom.twist.covariance[21] = unused
        odom.twist.covariance[28] = unused
        odom.twist.covariance[35] = self.odom_twist_cov_vyaw

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame_id
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node: Optional[RoboClawMecanumNode] = None
    try:
        node = RoboClawMecanumNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
