#!/usr/bin/env python3
"""Run an isolated RoboClaw velocity test and print a diagnostic report."""

import argparse
import csv
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray, Float64MultiArray


WHEELS = ("fl", "fr", "rl", "rr")
PID_OUTPUTS = ("left_m1", "left_m2", "right_m1", "right_m2")
STEP_STAGES = ((1, 0.1), (3, 0.2), (5, 0.3),
               (7, -0.1), (9, -0.2), (11, -0.3))
DRIVER_PARAMETERS = (
    "wheel_radius_m",
    "qpps_per_rad_per_sec",
    "max_wheel_rad_per_sec",
    "cmd_timeout_s",
    "front_left_output",
    "front_right_output",
    "rear_left_output",
    "rear_right_output",
)


class ValidationMonitor(Node):
    """Collect graph, parameter and latched diagnostic information."""

    def __init__(self):
        super().__init__("velocity_validation_monitor")
        self.pid_values = None
        self.latest_motor_diagnostics = None
        self.latest_battery_voltages = None
        pid_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Float64MultiArray,
            "/roboclaw/pid_config",
            self.pid_callback,
            pid_qos,
        )
        self.create_subscription(
            Float32MultiArray,
            "/roboclaw/motor_diagnostics",
            self.motor_diagnostics_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            "/roboclaw/battery_voltages",
            self.battery_callback,
            10,
        )

    def pid_callback(self, msg):
        if len(msg.data) >= 16:
            self.pid_values = list(msg.data[:16])

    def motor_diagnostics_callback(self, msg):
        if len(msg.data) >= 8:
            self.latest_motor_diagnostics = list(msg.data[:8])

    def battery_callback(self, msg):
        if len(msg.data) >= 2:
            self.latest_battery_voltages = list(msg.data[:2])

    def cmd_vel_publishers(self):
        return {
            self.full_node_name(info.node_namespace, info.node_name)
            for info in self.get_publishers_info_by_topic("/cmd_vel")
        }

    def node_names(self):
        return {
            self.full_node_name(namespace, name)
            for name, namespace in self.get_node_names_and_namespaces()
        }

    @staticmethod
    def full_node_name(namespace, name):
        prefix = namespace.rstrip("/")
        return f"{prefix}/{name}" if prefix else f"/{name}"

    def read_driver_parameters(self, timeout_s=5.0):
        client = AsyncParameterClient(self, "/roboclaw_mecanum_node")
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not client.services_are_ready():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not client.services_are_ready():
            raise RuntimeError("RoboClaw parameter service did not become ready")
        future = client.get_parameters(list(DRIVER_PARAMETERS))
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("Timed out while reading RoboClaw parameters")
        response = future.result()
        values = [parameter_value_to_python(value) for value in response.values]
        return dict(zip(DRIVER_PARAMETERS, values))


def parse_args():
    script_path = Path(__file__).resolve()
    workspace = script_path.parents[4]
    repo = script_path.parents[2]
    parser = argparse.ArgumentParser(
        description="Isolated RoboClaw velocity tracking validation",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Run without opening real RoboClaw serial ports.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the interactive physical-safety confirmation.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=repo / "sensors/amr_bringup/config/base.yaml",
    )
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=workspace / "velocity_logs",
    )
    return parser.parse_args()


def confirm_hardware_safety(skip_confirmation):
    if skip_confirmation:
        return
    print("\n[SAFETY] 실기 테스트 전 확인:")
    print("  - RoboClaw red fault 원인이 해결됨")
    print("  - 바퀴를 띄웠거나 충분한 주행 공간을 확보함")
    print("  - 비상정지/주전원 차단이 즉시 가능함")
    answer = input("계속하려면 RUN 을 입력하세요: ").strip()
    if answer != "RUN":
        raise RuntimeError("Safety confirmation was not accepted")


def start_process(command):
    print("[RUN]", " ".join(str(item) for item in command), flush=True)
    return subprocess.Popen(
        [str(item) for item in command],
        start_new_session=True,
    )


def stop_process(process, timeout_s=5.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=2.0)


def wait_for_driver(monitor, process, timeout_s=10.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"RoboClaw driver exited with code {process.returncode}")
        rclpy.spin_once(monitor, timeout_sec=0.1)
        if "/roboclaw_mecanum_node" in monitor.node_names():
            return
    raise RuntimeError("RoboClaw driver did not become ready")


def wait_for_pid_snapshot(monitor, require_valid=True, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and monitor.pid_values is None:
        rclpy.spin_once(monitor, timeout_sec=0.1)
    if monitor.pid_values is None:
        raise RuntimeError("RoboClaw PID/QPPS snapshot was not received")
    if require_valid and any(math.isnan(value) for value in monitor.pid_values):
        raise RuntimeError("One or more RoboClaw PID/QPPS reads failed")


def print_runtime_snapshot(parameters, pid_values):
    print("\n=== RUNTIME SNAPSHOT ===")
    print("ROS:")
    for name in DRIVER_PARAMETERS[:4]:
        print(f"  {name} = {parameters[name]}")
    print("Wheel mapping:")
    for wheel_name in ("front_left", "front_right", "rear_left", "rear_right"):
        print(f"  {wheel_name} -> {parameters[f'{wheel_name}_output']}")
    print("RoboClaw velocity PID/QPPS:")
    for index, output_name in enumerate(PID_OUTPUTS):
        p, i, d, qpps = pid_values[index * 4:(index + 1) * 4]
        if any(math.isnan(value) for value in (p, i, d, qpps)):
            print(f"  {output_name}: read unavailable")
            continue
        print(
            f"  {output_name}: P={p:.9g} I={i:.9g} "
            f"D={d:.9g} QPPS={int(qpps)}"
        )


def stable_rows(rows, stage):
    stage_rows = [row for row in rows if row["stage"] == str(stage)]
    if not stage_rows:
        return []
    first_time = min(float(row["time_s"]) for row in stage_rows)
    last_time = max(float(row["time_s"]) for row in stage_rows)
    return [
        row for row in stage_rows
        if first_time + 1.5 <= float(row["time_s"]) <= last_time - 0.2
        and all(abs(int(row[f"cmd_{wheel}_qpps"])) > 1 for wheel in WHEELS)
    ]


def analyze_csv(csv_path, parameters, publisher_names, mock_mode=False):
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise RuntimeError(f"CSV contains no data: {csv_path}")

    print("\n=== VELOCITY VALIDATION RESULT ===")
    print(f"CSV = {csv_path}")
    ratio_results = {}
    maximum_pwm = 0.0
    midstage_zero_rows = []
    for stage, velocity in STEP_STAGES:
        samples = stable_rows(rows, stage)
        if not samples:
            print(f"  {velocity:+.1f} m/s: insufficient stable samples")
            continue
        ratios = [
            int(row[f"meas_{wheel}_qpps"]) / int(row[f"cmd_{wheel}_qpps"])
            for row in samples for wheel in WHEELS
        ]
        ratio_median = statistics.median(ratios)
        ratio_mean = statistics.mean(ratios)
        odom_median = statistics.median(
            float(row["odom_vx_mps"]) for row in samples
        )
        pwm_values = [
            abs(float(row[f"pwm_{wheel}_percent"]))
            for row in samples for wheel in WHEELS
            if math.isfinite(float(row[f"pwm_{wheel}_percent"]))
        ]
        stage_maximum_pwm = max(pwm_values) if pwm_values else math.nan
        if pwm_values:
            maximum_pwm = max(maximum_pwm, stage_maximum_pwm)
        ratio_results[velocity] = ratio_median
        print(
            f"  {velocity:+.1f} m/s: measured/cmd median={ratio_median:.4f}, "
            f"mean={ratio_mean:.4f}, odom_vx={odom_median:.5f}, "
            f"max|PWM|={stage_maximum_pwm:.2f}%"
        )

        stage_rows = [row for row in rows if row["stage"] == str(stage)]
        stage_start = min(float(row["time_s"]) for row in stage_rows)
        midstage_zero_rows.extend(
            row for row in stage_rows
            if float(row["time_s"]) > stage_start + 0.1
            and all(int(row[f"cmd_{wheel}_qpps"]) == 0 for wheel in WHEELS)
        )

    radius = float(parameters["wheel_radius_m"])
    wheel_limit = float(parameters["max_wheel_rad_per_sec"])
    required_maximum = max(abs(velocity) for _, velocity in STEP_STAGES) / radius
    ros_saturation = required_maximum > wheel_limit
    output_saturation = maximum_pwm >= 95.0 if maximum_pwm else None
    print("\ncmd_vel publishers observed:")
    print(f"  count = {len(publisher_names)}")
    print(f"  node names = {', '.join(sorted(publisher_names))}")
    print("saturation:")
    print(f"  ROS wheel speed saturation = {'yes' if ros_saturation else 'no'}")
    if output_saturation is None:
        print("  RoboClaw motor output saturation = unknown")
    else:
        print(
            "  RoboClaw motor output saturation = "
            f"{'yes' if output_saturation else 'no'} "
            f"(maximum |PWM|={maximum_pwm:.2f}%)"
        )
    print(f"  unexpected mid-stage zero command rows = {len(midstage_zero_rows)}")

    worst_error = max(
        (abs(1.0 - ratio) for ratio in ratio_results.values()),
        default=math.inf,
    )
    print("\n판정:")
    if mock_mode:
        print("  mock 검증 완료: 실제 encoder/PWM 판정은 수행하지 않음")
    elif len(publisher_names) != 1 or publisher_names != {"/velocity_step_test"}:
        print("  E. command publisher 충돌 가능성이 가장 높음")
    elif midstage_zero_rows:
        print("  E. timeout 또는 외부 zero command를 우선 조사")
    elif output_saturation:
        print("  C. motor output saturation / 전압 / 부하를 우선 조사")
    elif worst_error <= 0.03:
        print("  정상: raw QPPS tracking 오차가 3% 이내임")
    else:
        print("  A/B. PID integral/feedforward와 internal QPPS를 우선 조사")


def main():
    args = parse_args()
    if not args.config.is_file():
        raise FileNotFoundError(f"Config file not found: {args.config}")
    args.output_directory.mkdir(parents=True, exist_ok=True)
    if not args.mock:
        confirm_hardware_safety(args.yes)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    monitor = ValidationMonitor()
    driver_process = None
    test_process = None
    observed_test_publishers = set()
    before_csv = set(args.output_directory.glob("velocity_step_test_*.csv"))
    try:
        for _ in range(5):
            rclpy.spin_once(monitor, timeout_sec=0.1)
        existing_publishers = monitor.cmd_vel_publishers()
        conflicting_nodes = monitor.node_names().intersection({
            "/roboclaw_mecanum_node",
            "/velocity_step_test",
            "/joy_teleop",
        })
        if existing_publishers or conflicting_nodes:
            raise RuntimeError(
                "Preflight failed. Stop the existing full system first. "
                f"cmd_vel publishers={sorted(existing_publishers)}, "
                f"conflicting nodes={sorted(conflicting_nodes)}"
            )

        driver_command = [
            "ros2", "run", "amr_base", "roboclaw_mecanum_node",
            "--ros-args", "--params-file", args.config,
            "-p", f"mock_mode:={'true' if args.mock else 'false'}",
        ]
        driver_process = start_process(driver_command)
        wait_for_driver(monitor, driver_process)
        parameters = monitor.read_driver_parameters()
        wait_for_pid_snapshot(monitor, require_valid=not args.mock)
        print_runtime_snapshot(parameters, monitor.pid_values)

        if not args.mock and not math.isclose(
            float(parameters["wheel_radius_m"]),
            0.04,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise RuntimeError(
                "wheel_radius_m does not match the stated physical radius 0.04 m"
            )
        if monitor.cmd_vel_publishers():
            raise RuntimeError("Unexpected cmd_vel publisher appeared before the test")

        test_command = [
            "ros2", "run", "amr_base", "velocity_step_test",
            "--ros-args", "-p", f"output_directory:={args.output_directory}",
        ]
        test_process = start_process(test_command)
        while test_process.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            if test_process.poll() is not None:
                break
            publishers = {
                name for name in monitor.cmd_vel_publishers()
                if "_NODE_NAME_UNKNOWN_" not in name
            }
            observed_test_publishers.update(publishers)
            unexpected = publishers.difference({"/velocity_step_test"})
            if unexpected or len(publishers) > 1:
                raise RuntimeError(
                    f"Unexpected /cmd_vel publisher during test: {sorted(publishers)}"
                )
        if test_process.returncode != 0:
            raise RuntimeError(
                f"velocity_step_test exited with code {test_process.returncode}"
            )
        test_process = None
        stop_process(driver_process)
        driver_process = None

        after_csv = set(args.output_directory.glob("velocity_step_test_*.csv"))
        new_csv = sorted(after_csv - before_csv, key=lambda path: path.stat().st_mtime)
        if not new_csv:
            raise RuntimeError("The test completed but no new CSV was found")
        analyze_csv(
            new_csv[-1],
            parameters,
            observed_test_publishers,
            mock_mode=args.mock,
        )
        if monitor.latest_battery_voltages is not None:
            print(
                "battery voltage snapshot: "
                + ", ".join(f"{value:.2f} V" for value in monitor.latest_battery_voltages)
            )
        return 0
    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C received; sending zero-command shutdown.")
        return 130
    except Exception as exc:
        print(f"\n[ABORT] {exc}", file=sys.stderr)
        return 1
    finally:
        # Stop the command publisher first so its finally path sends repeated
        # zero Twist messages while the motor driver is still available.
        stop_process(test_process)
        stop_process(driver_process)
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
