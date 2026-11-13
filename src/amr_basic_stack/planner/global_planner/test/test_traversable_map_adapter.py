"""OccupancyGrid -> Theta* 격자 변환 테스트.

여기서 잡으려는 버그는 상하 반전 하나다. OccupancyGrid 는 row 0 이 최소 y,
이 planner 의 내부 배열은 row 0 이 최대 y 다. 대칭적인 맵으로는 절대 안 잡히니
비대칭 단일 셀로 검증한다.
"""

from global_planner.common.ros_map_loader import RosMap
from global_planner.common.traversable_map import (
    geometry_matches,
    inflate_layer,
    traversable_obstacle_mask,
)
from global_planner.theta_star.costmap import Costmap

import numpy as np

import pytest

FREE = 0
BLOCKED = 100
UNKNOWN = -1


class _Origin:
    def __init__(self, x, y):
        self.position = type('P', (), {'x': x, 'y': y, 'z': 0.0})()


class _Info:
    def __init__(self, width, height, resolution, origin_x, origin_y):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = _Origin(origin_x, origin_y)


class _Grid:
    """테스트용 최소 OccupancyGrid. rclpy 없이 돌기 위한 것."""

    def __init__(self, values, resolution=0.5, origin_x=0.0, origin_y=0.0):
        values = np.asarray(values, dtype=np.int8)
        self.info = _Info(
            values.shape[1], values.shape[0], resolution, origin_x, origin_y,
        )
        self.data = values.reshape(-1).tolist()


def test_row_zero_of_the_wire_grid_becomes_the_last_image_row():
    """격자 row 0(최소 y)은 이미지 배열의 마지막 행이 되어야 한다."""
    grid = _Grid([
        [BLOCKED, FREE],     # row 0 = 최소 y
        [FREE, FREE],
        [FREE, FREE],        # row 2 = 최대 y
    ])
    mask = traversable_obstacle_mask(grid, unknown_is_obstacle=False)
    assert mask.tolist() == [
        [False, False],
        [False, False],
        [True, False],       # 이미지 마지막 행 = 최소 y
    ]


def test_conversion_agrees_with_the_loader_world_to_grid():
    """같은 world 좌표가 두 경로에서 같은 셀을 가리켜야 한다.

    RosMap.world_to_grid 는 planner 가 실제로 쓰는 변환이다. wire 격자에서
    장애물을 찍은 world 점을 그 변환에 넣었을 때 마스크가 True 여야 한다.
    """
    width, height, resolution = 4, 3, 0.5
    origin_x, origin_y = -1.0, -2.0

    values = np.full((height, width), FREE, dtype=np.int8)
    values[0, 3] = BLOCKED          # 최소 y, 최대 x

    grid = _Grid(values, resolution, origin_x, origin_y)
    mask = traversable_obstacle_mask(grid, unknown_is_obstacle=False)

    ros_map = RosMap(
        grid_map=np.zeros((height, width), dtype=int),
        image=np.zeros((height, width), dtype=np.uint8),
        image_path='',
        resolution=resolution,
        origin=(origin_x, origin_y, 0.0),
    )
    # 그 셀의 중심 world 좌표
    world = (origin_x + 3.5 * resolution, origin_y + 0.5 * resolution)
    cell_x, image_y = ros_map.world_to_grid(world)
    assert mask[image_y, cell_x], 'world_to_grid 와 flip 규약이 어긋났다'

    free_world = (origin_x + 0.5 * resolution, origin_y + 2.5 * resolution)
    free_x, free_image_y = ros_map.world_to_grid(free_world)
    assert not mask[free_image_y, free_x]


@pytest.mark.parametrize(
    'unknown_is_obstacle, expected',
    [
        (False, False),      # global planner: 미확인은 통과 허용
        (True, True),        # local planner 정책을 켠 경우
    ],
)
def test_unknown_policy_is_a_parameter(unknown_is_obstacle, expected):
    grid = _Grid([[UNKNOWN]])
    mask = traversable_obstacle_mask(grid, unknown_is_obstacle)
    assert bool(mask[0, 0]) is expected


def test_wire_value_one_would_not_be_an_obstacle():
    """1 은 장애물이 아니다. 그래서 wire 규약이 100 인 것이다.

    이 테스트가 깨지면 누군가 threshold 를 바꾼 것이고, traversability_node
    쪽 encoding.BLOCKED 와 함께 봐야 한다.
    """
    grid = _Grid([[1, BLOCKED]])
    mask = traversable_obstacle_mask(grid, unknown_is_obstacle=False)
    assert mask.tolist() == [[False, True]]


def test_geometry_mismatch_is_detected():
    ros_map = RosMap(
        grid_map=np.zeros((3, 4), dtype=int),
        image=np.zeros((3, 4), dtype=np.uint8),
        image_path='',
        resolution=0.5,
        origin=(-1.0, -2.0, 0.0),
    )
    assert geometry_matches(_Grid(np.zeros((3, 4)), 0.5, -1.0, -2.0).info,
                            ros_map)
    assert not geometry_matches(_Grid(np.zeros((3, 5)), 0.5, -1.0, -2.0).info,
                                ros_map)
    assert not geometry_matches(_Grid(np.zeros((3, 4)), 0.05, -1.0, -2.0).info,
                                ros_map)
    assert not geometry_matches(_Grid(np.zeros((3, 4)), 0.5, 0.0, -2.0).info,
                                ros_map)


def test_traversable_inflation_matches_static_costmap():
    """같은 장애물은 static map과 traversable map에서 똑같이 팽창해야 한다."""
    resolution = 0.05
    robot_size = 0.30
    raw = np.zeros((40, 40), dtype=int)
    raw[:, 20] = 1                       # 세로벽 하나
    raw[0, :] = raw[-1, :] = 1           # 위아래 테두리

    size_cells = int(np.ceil(robot_size / resolution))
    base = Costmap(raw, robot_size=size_cells)

    layer = inflate_layer(raw, robot_size, 0.0, resolution)

    assert np.array_equal(layer, base.inflated_map.astype(bool))


def test_inflate_layer_subtracts_what_the_producer_already_applied():
    """inflation_radius 가 채워져 있으면 그만큼 덜 팽창한다."""
    raw = np.zeros((21, 21), dtype=int)
    raw[10, 10] = 1

    full = inflate_layer(raw, 0.30, 0.0, 0.05)
    partial = inflate_layer(raw, 0.30, 0.10, 0.05)
    none = inflate_layer(raw, 0.30, 0.15, 0.05)

    assert int(full.sum()) > int(partial.sum()) > int(none.sum())
    assert int(none.sum()) == 1, '이미 다 팽창됐으면 더하지 않는다'


def test_inflate_layer_grows_a_new_obstacle_by_the_footprint():
    """기준 맵에 없던 장애물은 로봇 폭만큼 팽창되어야 한다."""
    raw = np.zeros((21, 21), dtype=int)
    raw[10, 10] = 1
    grown = inflate_layer(raw, 0.30, 0.0, 0.05)
    # 6 셀 정사각 커널 -> 6x6 = 36 셀
    assert int(grown.sum()) == 36


def test_a_layer_carrying_100_must_be_boolean_before_reaching_costmap():
    """Costmap.is_collision 은 `== 1` 이다. 100 을 그대로 넣으면 free 가 된다."""
    raw = np.array([[0, 100]], dtype=np.uint8)
    passthrough = Costmap(raw, robot_size=1)      # robot_size<=1 -> 팽창 안 함
    assert not passthrough.is_collision(1, 0), (
        '이 성질이 바뀌면 _planning_costmap 의 bool 변환을 재검토해야 한다'
    )

    folded = Costmap((raw > 0).astype(np.uint8), robot_size=1)
    assert folded.is_collision(1, 0)
