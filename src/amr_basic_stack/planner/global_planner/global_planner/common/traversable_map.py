"""Traversable map(wire) 격자를 이 planner 의 내부 표현으로 바꾼다.

여기는 ROS 메시지 타입에 의존하지 않는다 — info/data 속성만 읽는다. 덕분에
amr_interfaces 를 빌드하지 않고도 변환 규약을 테스트할 수 있다.

층 구분:
  층2 wire  nav_msgs/OccupancyGrid.data, int8, 0 / 100 / -1, row 0 = 최소 y
  층3 내부  bool 또는 0/1, row 0 = 최대 y (PGM 이미지 순서)
"""

import math

import numpy as np

from scipy.ndimage import binary_dilation

# nav2 map_server 의 occupied_thresh 0.65, DistanceField 의 occupied_threshold
# 65 와 같은 경계다. traversability 패키지의 encoding.OCCUPIED_THRESHOLD 와
# 같은 값이어야 한다.
OCCUPIED_THRESHOLD = 65


def traversable_obstacle_mask(grid, unknown_is_obstacle):
    """Convert a traversable-map OccupancyGrid into an image-order mask.

    Two conversions happen here and both are easy to get wrong.

    OccupancyGrid rows start at the minimum y, while this planner indexes its
    map in image order where row 0 is the maximum y.  The vertical flip below
    is the whole reason this function exists; without it the planner works on
    an upside-down map, which symmetric indoor maps hide completely.

    The value test uses the ROS occupancy convention rather than the 0/1 the
    planner uses internally.  The wire format carries 0 / 100 / -1, so a plain
    `!= 0` test would classify unknown as an obstacle regardless of policy.
    """
    shape = (grid.info.height, grid.info.width)
    values = np.asarray(grid.data, dtype=np.int8).reshape(shape)
    mask = values >= OCCUPIED_THRESHOLD
    if unknown_is_obstacle:
        mask |= values < 0
    return np.flipud(mask)


def inflate_layer(mask, robot_size, already_inflated, resolution):
    """Dilate a traversable-map layer the way Costmap dilates the static map.

    Costmap uses a square kernel whose side is the footprint *width* in cells,
    and scipy places an even-sided square off-centre - it grows by side//2 in
    the negative direction but only side//2 - 1 in the positive one.  Matching
    that exact kernel keeps traversable-map obstacle footprints consistent
    with static-map planning. A circular kernel of the same nominal size does
    not: it is symmetric, so it overshoots on the positive side.

    Getting this wrong is not cosmetic.  Over-inflating buries the robot's own
    start cell inside the safety margin and Theta* then rejects every goal with
    "start is inside an obstacle or inflated safety margin".

    ``already_inflated`` is the radius the producer says it applied, so the
    width still owed is ``robot_size - 2 * already_inflated``.
    """
    mask = np.asarray(mask, dtype=bool)
    owed = float(robot_size) - 2.0 * float(already_inflated)
    side = int(math.ceil(owed / float(resolution)))
    if side <= 1:
        return mask.copy()
    return binary_dilation(mask, structure=np.ones((side, side)))


def geometry_matches(info, ros_map, position_tolerance=1.0e-6):
    """Wire 격자의 기하가 이 노드가 읽은 PGM 과 같은지 확인한다."""
    return (
        info.width == ros_map.width
        and info.height == ros_map.height
        # ROS resolution is float32; YAML uses Python float64. A 0.1 m grid
        # differs by ~1.5e-9 after wire conversion even when it is identical.
        and abs(info.resolution - ros_map.resolution) <= max(
            1.0e-9, abs(ros_map.resolution) * 1.0e-6)
        and abs(info.origin.position.x - ros_map.origin[0]) <= position_tolerance
        and abs(info.origin.position.y - ros_map.origin[1]) <= position_tolerance
    )
