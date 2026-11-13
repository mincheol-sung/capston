import math

import numpy as np
from sensor_msgs.msg import LaserScan

from amr_laser_tools.scan_sector_filter import filter_scan


def make_scan():
    msg = LaserScan()
    msg.header.stamp.sec = 123
    msg.header.stamp.nanosec = 456
    msg.header.frame_id = 'laser_test'
    msg.angle_min = -math.pi
    msg.angle_max = math.pi
    msg.angle_increment = math.pi / 2.0
    msg.time_increment = 0.0125
    msg.scan_time = 0.05
    msg.range_min = 0.15
    msg.range_max = 30.0
    msg.ranges = [1.0, 1.0, 1.0, 1.0]
    msg.intensities = [1.0, 2.0, 3.0, 4.0]
    return msg


def test_front_sector_preserves_timing_and_masks_rear_beams():
    source = make_scan()
    result = filter_scan(
        source,
        transform=(0.0, 0.0, 0.0),
        sector=(-math.pi / 2.0, math.pi / 2.0),
    )

    assert result.header == source.header
    assert result.scan_time == source.scan_time
    assert result.time_increment == source.time_increment
    assert math.isinf(result.ranges[0])
    assert np.allclose(result.ranges[1:], [1.0, 1.0, 1.0])
    assert result.intensities[0] == 0.0
    assert list(source.ranges) == [1.0, 1.0, 1.0, 1.0]


def test_wraparound_rear_sector():
    result = filter_scan(
        make_scan(),
        transform=(0.0, 0.0, 0.0),
        sector=(math.pi / 2.0, -math.pi / 2.0),
    )

    assert math.isfinite(result.ranges[0])
    assert math.isfinite(result.ranges[1])
    assert math.isinf(result.ranges[2])
    assert math.isfinite(result.ranges[3])
