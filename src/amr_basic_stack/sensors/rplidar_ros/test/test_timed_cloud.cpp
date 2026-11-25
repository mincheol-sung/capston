#include <gtest/gtest.h>
#include <limits>

#include "timed_cloud.hpp"

TEST(TimedCloud, ReversesAndRotatesWithoutLosingBinsOrCompressingTime)
{
  for (size_t n : {5, 6}) {
    for (bool reverse : {false, true}) {
      for (bool flip : {false, true}) {
        sensor_msgs::msg::LaserScan scan;
        scan.header.frame_id = "laser";
        scan.header.stamp.sec = 123;
        scan.range_min = 1;
        scan.range_max = 6;
        scan.angle_min = -.8f;
        scan.angle_increment = .3f;
        scan.time_increment = .02f;
        for (size_t j = 0; j < n; ++j) {
          scan.ranges.push_back(j + 1);
          scan.intensities.push_back(10 + j);
        }
        scan.ranges[1] = std::numeric_limits<float>::infinity();
        scan.ranges[3] = .1f;
        const auto cloud = rplidar_ros::timed_cloud(scan, reverse, flip);
        EXPECT_EQ(cloud.header, scan.header);
        ASSERT_EQ(cloud.width, n - 2);
        sensor_msgs::PointCloud2ConstIterator<uint32_t> index(cloud, "index");
        sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"),
          z(cloud, "z"), t(cloud, "time"), intensity(cloud, "intensity");
        float prev = -1;
        std::vector<bool> seen(n);
        for (size_t k = 0; k < cloud.width; ++k, ++index, ++x, ++y, ++z, ++t, ++intensity) {
          const size_t j = *index;
          ASSERT_LT(j, n);
          EXPECT_FALSE(seen[j]);
          seen[j] = true;
          const size_t before_flip = flip ? (j + n - n / 2) % n : j;
          const size_t i = reverse ? n - 1 - before_flip : before_flip;
          EXPECT_FLOAT_EQ(*t, i * scan.time_increment);
          EXPECT_GT(*t, prev);
          prev = *t;
          const double a = scan.angle_min + double(j) * scan.angle_increment;
          EXPECT_FLOAT_EQ(*x, scan.ranges[j] * std::cos(a));
          EXPECT_FLOAT_EQ(*y, scan.ranges[j] * std::sin(a));
          EXPECT_FLOAT_EQ(*z, 0);
          EXPECT_FLOAT_EQ(*intensity, scan.intensities[j]);
        }
        EXPECT_FALSE(seen[1]);
        EXPECT_FALSE(seen[3]);
      }
    }
  }
}

TEST(TimedCloud, EmptyOrOutOfRangeScanHasLayoutAndNoReturns)
{
  sensor_msgs::msg::LaserScan scan;
  for (int i = 0; i < 2; ++i) {
    const auto cloud = rplidar_ros::timed_cloud(scan, true, true);
    EXPECT_EQ(cloud.height, 1u);
    EXPECT_EQ(cloud.width, 0u);
    EXPECT_TRUE(cloud.data.empty());
    EXPECT_GT(cloud.point_step, 0u);
    scan.ranges = {std::numeric_limits<float>::quiet_NaN(), 10.f};
    scan.range_max = 8;
  }
}
