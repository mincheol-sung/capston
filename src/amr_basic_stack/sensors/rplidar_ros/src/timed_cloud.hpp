#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace rplidar_ros
{
// i is the index entering publish_scan, after SDK sorting / angle compensation.
inline std::size_t scan_index(std::size_t i, std::size_t n, bool reverse_data, bool flip_x_axis)
{
  const auto j = reverse_data ? n - 1 - i : i;
  return flip_x_axis ? (j + n / 2) % n : j;
}

struct TimedBin
{
  std::size_t index;
  float seconds_from_start;
};

inline sensor_msgs::msg::PointCloud2 timed_cloud(
  const sensor_msgs::msg::LaserScan & scan, const std::vector<TimedBin> & bins)
{
  using Field = sensor_msgs::msg::PointField;
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = scan.header;
  cloud.height = 1;
  cloud.is_dense = true;
  const uint16_t one = 1;
  cloud.is_bigendian = *reinterpret_cast<const uint8_t *>(&one) == 0;
  sensor_msgs::PointCloud2Modifier layout(cloud);
  layout.setPointCloud2Fields(6, "x", 1, Field::FLOAT32, "y", 1, Field::FLOAT32,
    "z", 1, Field::FLOAT32, "intensity", 1, Field::FLOAT32, "time", 1, Field::FLOAT32,
    "index", 1, Field::UINT32);
  const auto n = scan.ranges.size();
  if (bins.empty()) {
    return cloud;
  }
  layout.resize(bins.size());
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
    intensity(cloud, "intensity"), time(cloud, "time");
  sensor_msgs::PointCloud2Iterator<uint32_t> index(cloud, "index");
  std::size_t count = 0;
  for (const auto & bin : bins) {
    const auto j = bin.index;
    const float range = scan.ranges[j];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = scan.angle_min + j * static_cast<double>(scan.angle_increment);
    *x = range * std::cos(angle);
    *y = range * std::sin(angle);
    *z = 0.f;
    *intensity = scan.intensities.size() == n ? scan.intensities[j] : 0.f;
    *time = bin.seconds_from_start;
    *index = j;  // LaserScan bin for consumers that mask individual beams.
    ++x; ++y; ++z; ++intensity; ++time; ++index;
    ++count;
  }
  layout.resize(count);
  return cloud;
}

// LaserScan-only replay: these messages do not contain the SDK's sample timestamps.
inline sensor_msgs::msg::PointCloud2 timed_cloud(
  const sensor_msgs::msg::LaserScan & scan, bool reverse_data, bool flip_x_axis)
{
  const auto n = scan.ranges.size();
  std::vector<TimedBin> bins;
  bins.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    bins.push_back({scan_index(i, n, reverse_data, flip_x_axis), i * scan.time_increment});
  }
  return timed_cloud(scan, bins);
}
}  // namespace rplidar_ros
