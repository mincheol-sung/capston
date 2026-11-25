"""Select complete PointCloud2 records in their incoming order."""

from array import array
import copy

import numpy as np
from sensor_msgs_py.point_cloud2 import dtype_from_fields


def records(cloud):
    """View the records in place, respecting byte order and row padding."""
    dtype = dtype_from_fields(cloud.fields, point_step=cloud.point_step)
    return np.ndarray((cloud.height, cloud.width),
                      dtype=dtype.newbyteorder('>' if cloud.is_bigendian else '<'),
                      buffer=cloud.data, strides=(cloud.row_step, cloud.point_step))


def select_records(cloud, points, keep):
    """Keep all fields and padding bytes of selected points."""
    out = copy.copy(cloud)
    out.height = 1
    out.width = int(np.count_nonzero(keep))
    out.row_step = out.width * out.point_step
    out.data = array('B', points.view(np.dtype((np.void, cloud.point_step)))[keep].tobytes())
    return out
