// Replay LaserScan-only bags with uniform timing; SDK timestamps exist only in recorded clouds.
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "timed_cloud.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("scan_to_timed_cloud");
  const bool reverse = node->declare_parameter("reverse_data", true);
  const bool flip = node->declare_parameter("flip_x_axis", true);
  auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_timed", 10);
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
    [pub, reverse, flip](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
      pub->publish(rplidar_ros::timed_cloud(*scan, reverse, flip));
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
