"""Time, extra fields and padding survive both kinds of beam masks."""

from array import array
import math
import struct

from amr_laser_tools.masked_timed_cloud import apply_mask
from amr_laser_tools.scan_sector_filter import filter_cloud
import numpy as np
import pytest
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


def organized_cloud():
    names = ('x', 'y', 'index', 'time')
    fields = [PointField(name=name, offset=i * 4, count=1,
                         datatype=PointField.UINT32 if name == 'index' else PointField.FLOAT32)
              for i, name in enumerate(names)]
    cloud = PointCloud2(height=2, width=2, fields=fields, is_bigendian=True,
                        point_step=17, row_step=36, is_dense=True)
    cloud.header.frame_id = 'laser'
    cloud.header.stamp.sec = 123
    data = bytearray(72)
    for i, (x, y, j) in enumerate([(1, 0, 3), (-1, 0, 2), (0, 1, 1), (-2, 0, 0)]):
        struct.pack_into('>ffIfB', data, i // 2 * 36 + i % 2 * 17, x, y, j, .01 * i, 90 + i)
    cloud.data = array('B', data)
    return cloud


def test_cloud_filter_keeps_complete_records_in_order():
    cloud = organized_cloud()
    out = filter_cloud(cloud, (0.25, 0.0, math.pi), (math.radians(95), math.radians(-95)))
    assert out.header == cloud.header and out.fields == cloud.fields and out.is_bigendian
    assert out.width == 1 and out.height == 1 and out.row_step == cloud.point_step
    assert out.data == cloud.data[:17]
    all_points = filter_cloud(cloud, (0, 0, 0), (-math.pi, math.pi))
    assert all_points.data == cloud.data[:34] + cloud.data[36:70]
    empty = filter_cloud(cloud, (0, 0, 0), (.1, .2))
    assert empty.width == 0 and empty.row_step == 0 and empty.header == cloud.header
    assert filter_cloud(empty, (0, 0, 0), (-math.pi, math.pi)).width == 0


def test_dynamic_mask_uses_original_bin_and_header():
    cloud = organized_cloud()
    mask = LaserScan(header=cloud.header, range_min=.1, range_max=8.,
                     ranges=[1.0, np.inf, 3.0, np.inf])
    out = apply_mask(cloud, mask)
    assert out.data == cloud.data[17:34] + cloud.data[53:70]
    mask.header = organized_cloud().header
    mask.header.frame_id = 'different'
    with pytest.raises(ValueError, match='same scan'):
        apply_mask(cloud, mask)
