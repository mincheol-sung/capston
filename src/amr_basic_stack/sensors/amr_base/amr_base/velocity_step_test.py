import csv
from datetime import datetime
import math
from pathlib import Path
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32MultiArray


WHEEL_NAMES = ("fl", "fr", "rl", "rr")
DRIVE_PARAMETER_NAMES = (
    "wheel_radius_m",
    "qpps_per_rad_per_sec",
    "max_wheel_rad_per_sec",
)


class VelocityStepTest(Node):
    """Publish a longitudinal velocity step sequence and log base feedback."""

    def __init__(self):
        super().__init__("velocity_step_test")

        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("output_directory", ".")
        self.declare_parameter("driver_node_name", "/roboclaw_mecanum_node")
        self.declare_parameter("stop_publish_count", 10)
        self.declare_parameter("qpps_zero_threshold", 1)
        self.declare_parameter(
            "step_velocities_mps",
            [0.0, 0.1, 0.0, 0.2, 0.0, 0.3, 0.0,
             -0.1, 0.0, -0.2, 0.0, -0.3, 0.0],
        )
        self.declare_parameter(
            "step_durations_s",
            [2.0, 4.0, 2.0, 4.0, 2.0, 4.0, 2.0,
             4.0, 2.0, 4.0, 2.0, 4.0, 2.0],
        )

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.stop_publish_count = int(self.get_parameter("stop_publish_count").value)
        self.qpps_zero_threshold = int(self.get_parameter("qpps_zero_threshold").value)
        velocities = list(self.get_parameter("step_velocities_mps").value)
        durations = list(self.get_parameter("step_durations_s").value)
        if self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.stop_publish_count < 2:
            raise ValueError("stop_publish_count must be at least 2")
        if len(velocities) != len(durations) or not velocities:
            raise ValueError("step velocities and durations must have equal non-zero lengths")
        if any(duration <= 0.0 for duration in durations):
            raise ValueError("all step durations must be greater than zero")
        self.sequence = list(zip(velocities, durations))
        self.total_duration_s = sum(durations)

        output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser().resolve()
        output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = output_directory / f"velocity_step_test_{timestamp}.csv"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(self.csv_columns())

        self.command_wheels = [math.nan] * 4
        self.measured_wheels = [math.nan] * 4
        self.command_qpps: list[Optional[int]] = [None] * 4
        self.measured_qpps: list[Optional[int]] = [None] * 4
        self.odom_velocity = [math.nan] * 3
        self.motor_pwm_percent = [math.nan] * 4
        self.motor_current_a = [math.nan] * 4
        self.pid_config_received = False
        self.row_count = 0
        self.finished = False
        self.stopping = False
        self.stop_messages_remaining = self.stop_publish_count
        self.shutdown_complete = False

        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(
            Float32MultiArray,
            "wheel_speeds",
            self.command_wheels_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "measured_wheel_speeds",
            self.measured_wheels_callback,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            "roboclaw/command_qpps",
            self.command_qpps_callback,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            "roboclaw/measured_qpps",
            self.measured_qpps_callback,
            10,
        )
        pid_config_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Float64MultiArray,
            "roboclaw/pid_config",
            self.pid_config_callback,
            pid_config_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            "roboclaw/motor_diagnostics",
            self.motor_diagnostics_callback,
            10,
        )
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)

        driver_node_name = str(self.get_parameter("driver_node_name").value)
        self.parameter_client = AsyncParameterClient(self, driver_node_name)
        self.parameter_request_sent = False
        self.parameter_timer = self.create_timer(0.1, self.request_drive_parameters)

        self.start_monotonic = time.monotonic()
        self.test_timer = self.create_timer(1.0 / self.publish_rate_hz, self.test_tick)
        self.get_logger().info(f"Writing velocity step data to {self.csv_path}")
        self.get_logger().info(
            f"Starting {len(self.sequence)} stages at {self.publish_rate_hz:.1f} Hz; "
            f"sequence duration={self.total_duration_s:.1f} s"
        )

    @staticmethod
    def csv_columns():
        columns = ["time_s", "stage", "cmd_vx_mps"]
        columns.extend(f"cmd_{name}_rad_s" for name in WHEEL_NAMES)
        columns.extend(f"meas_{name}_rad_s" for name in WHEEL_NAMES)
        columns.extend(f"cmd_{name}_qpps" for name in WHEEL_NAMES)
        columns.extend(f"meas_{name}_qpps" for name in WHEEL_NAMES)
        columns.extend(["odom_vx_mps", "odom_vy_mps", "odom_wz_rad_s"])
        columns.extend(f"pwm_{name}_percent" for name in WHEEL_NAMES)
        columns.extend(f"current_{name}_a" for name in WHEEL_NAMES)
        columns.extend(f"ratio_{name}" for name in WHEEL_NAMES)
        return columns

    def command_wheels_callback(self, msg: Float32MultiArray):
        self.command_wheels = self.read_four_values(msg.data, self.command_wheels)

    def measured_wheels_callback(self, msg: Float32MultiArray):
        self.measured_wheels = self.read_four_values(msg.data, self.measured_wheels)

    def command_qpps_callback(self, msg: Int32MultiArray):
        self.command_qpps = self.read_four_values(msg.data, self.command_qpps)

    def measured_qpps_callback(self, msg: Int32MultiArray):
        self.measured_qpps = self.read_four_values(msg.data, self.measured_qpps)

    def pid_config_callback(self, msg: Float64MultiArray):
        if self.pid_config_received or len(msg.data) < 16:
            return
        self.pid_config_received = True
        output_names = ("left_m1", "left_m2", "right_m1", "right_m2")
        lines = ["RoboClaw PID/QPPS snapshot used for this step test:"]
        for index, output_name in enumerate(output_names):
            p, i, d, qpps = msg.data[index * 4:(index + 1) * 4]
            if any(math.isnan(value) for value in (p, i, d, qpps)):
                lines.append(f"  {output_name}: read unavailable")
            else:
                lines.append(
                    f"  {output_name}: P={p:.9g}, I={i:.9g}, "
                    f"D={d:.9g}, QPPS={int(qpps)}"
                )
        self.get_logger().info("\n".join(lines))

    def motor_diagnostics_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 8:
            return
        self.motor_pwm_percent = list(msg.data[:4])
        self.motor_current_a = list(msg.data[4:8])

    @staticmethod
    def read_four_values(values, previous):
        if len(values) < 4:
            return previous
        return list(values[:4])

    def odom_callback(self, msg: Odometry):
        self.odom_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.angular.z,
        ]

    def request_drive_parameters(self):
        if self.parameter_request_sent:
            return
        if not self.parameter_client.services_are_ready():
            return
        self.parameter_request_sent = True
        self.parameter_timer.cancel()
        self.parameter_client.get_parameters(
            list(DRIVE_PARAMETER_NAMES),
            callback=self.drive_parameters_callback,
        )

    def drive_parameters_callback(self, future):
        try:
            response = future.result()
            values = [parameter_value_to_python(value) for value in response.values]
            parameters = dict(zip(DRIVE_PARAMETER_NAMES, values))
            radius = float(parameters["wheel_radius_m"])
            qpps_scale = float(parameters["qpps_per_rad_per_sec"])
            max_wheel_speed = float(parameters["max_wheel_rad_per_sec"])
        except Exception as exc:
            self.get_logger().warning(f"Could not read drive parameters: {exc}")
            return

        self.get_logger().info(
            "RoboClaw runtime parameters: "
            f"wheel_radius_m={radius:.6g}, "
            f"qpps_per_rad_per_sec={qpps_scale:.6g}, "
            f"max_wheel_rad_per_sec={max_wheel_speed:.6g}"
        )
        if not math.isclose(radius, 0.04, rel_tol=0.0, abs_tol=1e-6):
            self.get_logger().warning(
                f"wheel_radius_m is {radius:.6g}, but the stated physical radius is 0.04 m"
            )
        for velocity in sorted({abs(value) for value, _ in self.sequence if value}):
            theoretical = velocity / radius
            saturation = theoretical > max_wheel_speed
            self.get_logger().info(
                f"|vx|={velocity:.3f} m/s -> {theoretical:.3f} rad/s; "
                f"saturation={'yes' if saturation else 'no'}"
            )

    def test_tick(self):
        elapsed = time.monotonic() - self.start_monotonic
        if self.stopping:
            self.publish_and_log_zero(elapsed)
            self.stop_messages_remaining -= 1
            if self.stop_messages_remaining <= 0:
                self.finished = True
                self.test_timer.cancel()
                self.close_csv()
                self.shutdown_complete = True
                self.get_logger().info(f"Velocity step test complete: {self.csv_path}")
            return

        stage, velocity = self.stage_at(elapsed)
        if stage is None:
            self.stopping = True
            self.publish_and_log_zero(elapsed)
            self.stop_messages_remaining -= 1
            return

        command = Twist()
        command.linear.x = velocity
        self.cmd_pub.publish(command)
        self.write_row(elapsed, stage, velocity)

    def stage_at(self, elapsed):
        stage_end = 0.0
        for stage, (velocity, duration) in enumerate(self.sequence):
            stage_end += duration
            if elapsed < stage_end:
                return stage, velocity
        return None, 0.0

    def publish_and_log_zero(self, elapsed):
        self.cmd_pub.publish(Twist())
        self.write_row(elapsed, "stop", 0.0)

    def write_row(self, elapsed, stage, velocity):
        ratios = []
        for command, measured in zip(self.command_qpps, self.measured_qpps):
            if command is None or measured is None or abs(command) <= self.qpps_zero_threshold:
                ratios.append(math.nan)
            else:
                ratios.append(float(measured) / float(command))

        qpps_values = ["" if value is None else value for value in self.command_qpps]
        measured_qpps = ["" if value is None else value for value in self.measured_qpps]
        self.csv_writer.writerow(
            [elapsed, stage, velocity]
            + self.command_wheels
            + self.measured_wheels
            + qpps_values
            + measured_qpps
            + self.odom_velocity
            + self.motor_pwm_percent
            + self.motor_current_a
            + ratios
        )
        self.row_count += 1
        if self.row_count % max(1, int(self.publish_rate_hz)) == 0:
            self.csv_file.flush()

    def close_csv(self):
        if not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

    def stop_and_close(self):
        if self.shutdown_complete:
            return
        for _ in range(self.stop_publish_count):
            try:
                self.cmd_pub.publish(Twist())
            except Exception:  # pragma: no cover - invalid ROS context
                break
            time.sleep(1.0 / self.publish_rate_hz)
        self.close_csv()
        self.shutdown_complete = True

    def destroy_node(self):
        self.stop_and_close()
        super().destroy_node()


def main(args=None):
    # Keep the ROS context alive during KeyboardInterrupt so the finally path
    # can deliver repeated zero commands before shutting middleware down.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[VelocityStepTest] = None
    try:
        node = VelocityStepTest()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
