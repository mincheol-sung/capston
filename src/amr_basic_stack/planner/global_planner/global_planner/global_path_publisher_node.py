import argparse
import csv
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


DEFAULT_PATH_TOPIC = "/global_path"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Republish generated global waypoints as nav_msgs/Path."
    )
    parser.add_argument(
        "map",
        nargs="?",
        default="my_factory_map",
        help="Map name under maps/<map_name>.",
    )
    parser.add_argument(
        "--map-dir",
        default="",
        help="Directory containing <map_name>/waypoints.csv.",
    )
    parser.add_argument(
        "--waypoints-file",
        help="Direct path to waypoints.csv. Overrides --map-dir.",
    )
    parser.add_argument(
        "--path-topic",
        default=DEFAULT_PATH_TOPIC,
        help="Topic name for publishing the generated global path.",
    )
    parser.add_argument(
        "--frame-id",
        default="map",
        help="Frame id for the published nav_msgs/Path.",
    )
    parser.add_argument(
        "--publish-period",
        type=float,
        default=1.0,
        help="Path republish period in seconds while the node is alive.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish the generated path once and exit.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for waypoints.csv to appear.",
    )
    return parser.parse_args(argv)


def resolve_waypoints_file(args):
    if args.waypoints_file:
        return os.path.abspath(args.waypoints_file)
    if not args.map_dir:
        raise ValueError("--map-dir is required when --waypoints-file is not given.")
    return os.path.join(os.path.abspath(args.map_dir), args.map, "waypoints.csv")


def wait_for_file(path, timeout):
    deadline = time.monotonic() + timeout
    while not os.path.isfile(path):
        if time.monotonic() >= deadline:
            raise FileNotFoundError(path)
        time.sleep(0.1)


def load_world_waypoints(path):
    waypoints = []
    with open(path, "r", newline="", encoding="utf-8") as file:
        reader = csv.reader(file)
        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue
            is_header = line_number == 1 and [
                value.strip().lower() for value in row
            ] == ["x", "y"]
            if is_header:
                continue
            if len(row) != 2:
                raise ValueError(
                    f"Invalid waypoint at line {line_number}: {row}"
                )
            try:
                waypoints.append((float(row[0]), float(row[1])))
            except ValueError as exc:
                raise ValueError(
                    f"Invalid waypoint at line {line_number}: {row}"
                ) from exc

    if not waypoints:
        raise RuntimeError(f"No waypoints found in {path}")
    return waypoints


class GlobalPathPublisher(Node):
    def __init__(self, path_topic, frame_id, world_path):
        super().__init__("global_path_publisher_node")
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, path_topic, qos)
        self.path_topic = path_topic
        self.frame_id = frame_id
        self.path_msg = self.build_path_message(world_path)

    def build_path_message(self, world_path):
        msg = Path()
        msg.header.frame_id = self.frame_id
        for x, y in world_path:
            pose = PoseStamped()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        for pose in self.path_msg.poses:
            pose.header = self.path_msg.header
        self.path_pub.publish(self.path_msg)
        self.get_logger().info(
            f"Published {len(self.path_msg.poses)} global path poses on {self.path_topic}"
        )


def main(argv=None):
    args = parse_args(argv)
    try:
        waypoints_file = resolve_waypoints_file(args)
        wait_for_file(waypoints_file, args.wait_timeout)
        world_path = load_world_waypoints(waypoints_file)
    except (FileNotFoundError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1

    rclpy.init(args=None)
    node = GlobalPathPublisher(args.path_topic, args.frame_id, world_path)
    try:
        node.get_logger().info(f"Loaded global path from {waypoints_file}")
        node.publish_path()
        if args.once:
            rclpy.spin_once(node, timeout_sec=0.5)
        else:
            node.create_timer(args.publish_period, node.publish_path)
            node.get_logger().info("Keeping global path publisher alive.")
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
