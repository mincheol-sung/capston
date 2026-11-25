#pragma once

#include <algorithm>
#include <limits>
#include <vector>

#include "sl_lidar_driver.h"
#include "timed_cloud.hpp"

namespace rplidar_ros
{
struct ScanTimeBounds
{
  sl_u64 start_us;
  sl_u64 end_us;
};

// Called on a complete, nonempty scan. USB buffering can reorder packet time estimates.
inline ScanTimeBounds scan_time_bounds(const sl::TimedHqNode * nodes, std::size_t count)
{
  const auto bounds = std::minmax_element(nodes, nodes + count, [](const auto & a, const auto & b) {
    return a.timestamp_us < b.timestamp_us;
  });
  return {bounds.first->timestamp_us, bounds.second->timestamp_us};
}

inline float angle_degrees(const sl::TimedHqNode & sample)
{
  return sample.node.angle_z_q14 * 90.f / 16384.f;
}

struct ProjectedScan
{
  sensor_msgs::msg::LaserScan laser;
  sensor_msgs::msg::PointCloud2 cloud;
};

// Each compensated bin is a copy of the winning measurement, including its SDK timestamp.
inline ProjectedScan project_sdk_scan(
  sl::TimedHqNode * nodes, std::size_t count, ScanTimeBounds bounds,
  const std_msgs::msg::Header & header, std::size_t bins_per_degree,
  bool inverted, bool flip_x_axis, float max_range)
{
  ProjectedScan out;
  auto & scan = out.laser;
  scan.header = header;
  scan.range_min = 0.15f;
  scan.range_max = max_range;
  scan.scan_time = (bounds.end_us - bounds.start_us) * 1e-6;
  float first_deg = 0.f;
  float last_deg = 359.f;
  std::vector<sl::TimedHqNode> compensated;
  if (sl::ascendTimedScanData(nodes, count) == SL_RESULT_OK) {
    if (bins_per_degree) {
      compensated.resize(360 * bins_per_degree);
      for (std::size_t i = 0; i < count; ++i) {
        if (!nodes[i].node.dist_mm_q2) continue;
        const auto first = static_cast<std::size_t>(angle_degrees(nodes[i]) * bins_per_degree);
        for (std::size_t k = 0; k < bins_per_degree; ++k) {
          compensated[std::min(first + k, compensated.size() - 1)] = nodes[i];
        }
      }
      nodes = compensated.data();
      count = compensated.size();
    } else {
      while (!nodes[0].node.dist_mm_q2) { ++nodes; --count; }
      while (!nodes[count - 1].node.dist_mm_q2) { --count; }
      first_deg = angle_degrees(nodes[0]);
      last_deg = angle_degrees(nodes[count - 1]);
    }
  }

  const auto first_angle = static_cast<float>(first_deg * M_PI / 180.);
  const auto last_angle = static_cast<float>(last_deg * M_PI / 180.);
  scan.angle_min = M_PI - std::max(first_angle, last_angle);
  scan.angle_max = M_PI - std::min(first_angle, last_angle);
  scan.angle_increment = count > 1 ? (scan.angle_max - scan.angle_min) / (count - 1.) : 0.f;
  scan.time_increment = count > 1 ? scan.scan_time / (count - 1.) : 0.f;
  scan.ranges.resize(count);
  scan.intensities.resize(count);
  const bool reverse_data = (last_angle > first_angle) != inverted;
  std::vector<TimedBin> bins;
  bins.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const auto & sample = nodes[i];
    const auto j = scan_index(i, count, reverse_data, flip_x_axis);
    scan.intensities[j] = sample.node.quality >> 2;
    scan.ranges[j] = sample.node.dist_mm_q2 ? sample.node.dist_mm_q2 / 4000.f :
      std::numeric_limits<float>::infinity();
    if (scan.ranges[j] >= scan.range_min && scan.ranges[j] <= scan.range_max) {
      bins.push_back({j, static_cast<float>((sample.timestamp_us - bounds.start_us) * 1e-6)});
    }
  }

  // Cartographer uses the last retained point as the cloud's time reference.
  std::stable_sort(bins.begin(), bins.end(), [](const TimedBin & a, const TimedBin & b) {
    return a.seconds_from_start < b.seconds_from_start;
  });
  out.cloud = timed_cloud(scan, bins);
  return out;
}
}  // namespace rplidar_ros
