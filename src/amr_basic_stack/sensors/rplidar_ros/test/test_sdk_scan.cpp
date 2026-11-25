#include <gtest/gtest.h>
#include <algorithm>
#include <map>
#include <memory>
#include <vector>
#include "sdk_scan.hpp"
#include "scan_data_holder.h"
#include <cartographer_ros/msg_conversion.h>
#include <cartographer_ros/time_conversion.h>

namespace
{
constexpr sl_u64 epoch_us = 9876543210000ULL;

sl::TimedHqNode sample(float degrees, int metres, int offset_us, int quality)
{
  sl::TimedHqNode s{};
  s.node.angle_z_q14 = static_cast<sl_u16>(degrees * 16384 / 90);
  s.node.dist_mm_q2 = metres * 4000;
  s.node.quality = quality * 4;
  s.timestamp_us = epoch_us + offset_us;
  return s;
}

// Acquisition crosses zero; SDK packet estimates can overlap at packet boundaries.
// Two near-zero rays compete for bin 1 when compensation is two bins per degree.
const std::vector<sl::TimedHqNode> original = {
  sample(359.75f, 5, 70, 5), sample(0.f, 1, 101, 1),
  sample(0.75f, 2, 45, 2), sample(40.f, 0, 0, 0),
  sample(135.f, 3, 1705, 3), sample(200.f, 31, 1910, 6)
};

void deliver(sl::ScanDataHolder & holder)
{
  for (std::size_t i = 0; i <= original.size(); ++i) {
    auto s = original[i % original.size()];
    s.node.flag = i % original.size() == 0 ? RPLIDAR_RESP_HQ_FLAG_SYNCBIT : 0;
    holder.pushScanNodeData(s.timestamp_us, &s.node);
  }
}

TEST(SdkScan, FullRevolutionPreservesDecodedPairsAndLegacyFirstTimestamp)
{
  sl::ScanDataHolder holder;
  deliver(holder);
  std::vector<sl::TimedHqNode> received(16);
  auto count = received.size();
  ASSERT_EQ(holder.readScan(nullptr, received.data(), count, nullptr, 10), SL_RESULT_OK);
  ASSERT_EQ(count, original.size());
  for (std::size_t i = 0; i < count; ++i) {
    EXPECT_EQ(received[i].timestamp_us, original[i].timestamp_us);
    EXPECT_EQ(received[i].node.dist_mm_q2, original[i].node.dist_mm_q2);
    EXPECT_EQ(received[i].node.angle_z_q14, original[i].node.angle_z_q14);
  }
  holder.rewindCurrentScanData();
  deliver(holder);
  sl_lidar_response_measurement_node_hq_t legacy[3];
  count = 3;
  sl_u64 first_us = 0;
  ASSERT_EQ(holder.readScan(legacy, nullptr, count, &first_us, 10), SL_RESULT_OK);
  EXPECT_EQ(first_us, original.front().timestamp_us);
  ASSERT_EQ(count, 3u);
  for (std::size_t i = 0; i < count; ++i) EXPECT_EQ(legacy[i].dist_mm_q2, original[i].node.dist_mm_q2);
}

TEST(SdkScan, CartographerRecoversWinningMeasurementsAfterSortCompensationAndFlip)
{
  for (std::size_t multiple : {0, 2}) {
    // Uncompensated SDK sorting includes invalid and out-of-range bins.
    const std::map<std::size_t, int> expected = multiple ?
      std::map<std::size_t, int>{{0,1}, {1,2}, {2,2}, {270,3}, {271,3}, {719,5}} :
      std::map<std::size_t, int>{{0,1}, {1,2}, {2,3}, {5,5}};
    for (bool inverted : {false, true}) {
      for (bool flip : {false, true}) {
        auto nodes = original;
        std_msgs::msg::Header header;
        header.frame_id = "laser";
        header.stamp.sec = 1700000000;
        header.stamp.nanosec = 123456789;
        const auto bounds = rplidar_ros::scan_time_bounds(nodes.data(), nodes.size());
        ASSERT_EQ(bounds.start_us, epoch_us);
        const auto projected = rplidar_ros::project_sdk_scan(
          nodes.data(), nodes.size(), bounds, header, multiple, inverted, flip, 30.f);
        const auto & cloud = projected.cloud;
        ASSERT_EQ(cloud.header, projected.laser.header);
        ASSERT_EQ(cloud.header, header);
        ASSERT_EQ(cloud.width, expected.size());
        const auto n = projected.laser.ranges.size();
        ASSERT_EQ(n, multiple ? 720u : 6u);
        auto converted = cartographer_ros::ToPointCloudWithIntensities(cloud);
        const auto & points = std::get<0>(converted);
        ASSERT_EQ(points.points.size(), expected.size());
        const auto end_ns = cartographer_ros::ToRos(std::get<1>(converted)).nanoseconds();
        sensor_msgs::PointCloud2ConstIterator<uint32_t> bin(cloud, "index");
        sensor_msgs::PointCloud2ConstIterator<float> t(cloud, "time");
        std::vector<bool> seen(n);
        float prev = -1;
        for (std::size_t k = 0; k < cloud.width; ++k, ++bin, ++t) {
          const auto j = *bin;
          ASSERT_LT(j, n);
          ASSERT_FALSE(seen[j]);
          seen[j] = true;
          const auto unflipped = flip ? (j + n - n / 2) % n : j;
          const auto i = inverted ? unflipped : n - 1 - unflipped;
          ASSERT_TRUE(expected.count(i)) << "bin " << i << ", compensation " << multiple;
          const int metres = expected.at(i);
          const auto & s = *std::find_if(original.begin(), original.end(), [metres](const auto & value) {
            return value.node.dist_mm_q2 == static_cast<unsigned>(metres * 4000);
          });
          EXPECT_FLOAT_EQ(projected.laser.ranges[j], metres);
          const auto & point = points.points[k];
          const double angle = projected.laser.angle_min + j * double(projected.laser.angle_increment);
          EXPECT_NEAR(point.position.x(), metres * std::cos(angle), 1e-6);
          EXPECT_NEAR(point.position.y(), metres * std::sin(angle), 1e-6);
          EXPECT_EQ(points.intensities[k], s.node.quality >> 2);
          EXPECT_GE(*t, prev);
          prev = *t;
          EXPECT_FLOAT_EQ(*t, (s.timestamp_us - epoch_us) * 1e-6f);
          const auto restored_ns = end_ns + std::llround(point.time * 1e9);
          const auto expected_ns = rclcpp::Time(header.stamp).nanoseconds() +
            static_cast<int64_t>(s.timestamp_us - epoch_us) * 1000;
          EXPECT_LE(std::abs(restored_ns - expected_ns), 200);
        }
        EXPECT_FLOAT_EQ(points.points.back().time, 0.f);
      }
    }
  }
}

TEST(SdkScan, IdenticalAnglesAcrossRevolutionKeepTheLegacyWinningDistanceAndItsTime)
{
  auto nodes = original;
  nodes.push_back(sample(359.75f, 7, 99010, 7));
  // Mark each valid return with its source index before running the original HQ sorter.
  std::vector<sl_lidar_response_measurement_node_hq_t> legacy;
  for (const auto & s : nodes) {
    auto marked = s.node;
    if (marked.dist_mm_q2) marked.dist_mm_q2 = legacy.size() + 1;
    legacy.push_back(marked);
  }
  std::unique_ptr<sl::ILidarDriver> driver(*sl::createLidarDriver());
  ASSERT_EQ(driver->ascendScanData(legacy.data(), legacy.size()), SL_RESULT_OK);
  const auto winner = nodes.at(legacy.back().dist_mm_q2 - 1);
  const auto bounds = rplidar_ros::scan_time_bounds(nodes.data(), nodes.size());
  const auto output = rplidar_ros::project_sdk_scan(
    nodes.data(), nodes.size(), bounds, std_msgs::msg::Header(), 2, false, true, 30.f);
  const std::size_t last_bin = rplidar_ros::scan_index(719, 720, true, true);
  EXPECT_FLOAT_EQ(output.laser.ranges[last_bin], winner.node.dist_mm_q2 / 4000.f);
  sensor_msgs::PointCloud2ConstIterator<uint32_t> index(output.cloud, "index");
  sensor_msgs::PointCloud2ConstIterator<float> time(output.cloud, "time");
  for (; index != index.end(); ++index, ++time) {
    if (*index == last_bin) {
      EXPECT_FLOAT_EQ(*time, (winner.timestamp_us - bounds.start_us) * 1e-6f);
      return;
    }
  }
  FAIL() << "Missing last angular bin";
}

TEST(SdkScan, InvalidScanAndSingleReturnRemainPublishable)
{
  for (int distance : {0, 4}) {
    auto raw = sample(30.f, distance, 0, 9);
    auto scan = rplidar_ros::project_sdk_scan(&raw, 1, {epoch_us, epoch_us}, std_msgs::msg::Header(), 0, false, true, 30.f);
    EXPECT_EQ(scan.laser.ranges.size(), 1u);
    EXPECT_FLOAT_EQ(scan.laser.time_increment, 0.f);
    EXPECT_EQ(scan.cloud.width, distance ? 1u : 0u);
  }
}
}  // namespace
