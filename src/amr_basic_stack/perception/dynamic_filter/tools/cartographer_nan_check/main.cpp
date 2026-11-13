// Exercise the installed ROS 2 port, not a reimplementation of its predicate.
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "cartographer_ros/msg_conversion.h"

void require(bool result, const char* message) {
  if (!result) throw std::runtime_error(message);
}

int main() {
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp.sec = 10;
  scan.header.frame_id = "laser";
  scan.angle_min = 0.f;
  scan.angle_max = 0.7f;
  scan.angle_increment = 0.1f;
  scan.time_increment = 0.01f;
  scan.range_min = 0.1f;
  scan.range_max = 10.f;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  scan.ranges = {1.f, nan, inf, -inf, 0.f, 11.f, 2.f, nan};
  scan.intensities = {10.f, 20.f, 30.f, 40.f, 50.f, 60.f, 70.f, 80.f};
  auto [cloud, stamp] = cartographer_ros::ToPointCloudWithIntensities(scan);
  require(cloud.points.size() == 2, "invalid observations became points");
  require(cloud.intensities.size() == 2, "intensity correspondence lost");
  require(cloud.intensities[0] == 10.f && cloud.intensities[1] == 70.f,
          "incorrect beam indices");
  require(std::abs(cloud.points[0].time + 0.06f) < 1e-6f,
          "beam times were compacted after NaN removal");
  require(cloud.points[1].time == 0.f, "last retained beam is not time origin");
  require(std::abs(cloud.points[1].position.norm() - 2.f) < 1e-6f,
          "invalid range generated a distant return");
  scan.ranges.assign(8, nan);
  auto [empty, empty_stamp] = cartographer_ros::ToPointCloudWithIntensities(scan);
  require(empty.points.empty() && empty.intensities.empty(), "all-NaN scan not empty");
  scan.ranges.clear();
  scan.intensities.clear();
  require(std::get<0>(cartographer_ros::ToPointCloudWithIntensities(scan)).points.empty(),
          "empty scan not empty");
  std::cout << "PASS: installed Cartographer skips NaN/Inf/out-of-range beams; "
               "retains indices/timing; all-NaN and empty scans give no points.\n";
}
