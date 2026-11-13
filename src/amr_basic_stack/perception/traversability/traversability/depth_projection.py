"""D455 점군의 z-band 필터와 2D 투영.

ROS 메시지에 의존하지 않는다 — numpy 배열과 스칼라만 받는다. 노드와 테스트가
같은 함수를 쓴다.

좌표 규약 (설계문서 §4.1):
  z 축은 **수직 위쪽**이며 원점은 height_frame(기본 base_footprint)이다.
  camera optical 좌표계의 z 는 전방 깊이이므로 그대로 쓰면 안 된다. 노드가
  TF 로 변환한 뒤 이 모듈에 넘긴다.

투영 결과 인덱스는 OccupancyGrid 순서(row 0 = 최소 y)다. 이미지 순서를 쓰는
소비자는 경계에서 flip 을 한 번 걸어야 한다.
"""

import numpy as np


def finite_points(points):
    """NaN/Inf 가 있는 행을 버린다.

    RealSense 는 깊이가 없는 픽셀을 0 또는 NaN 으로 낸다. ordered pointcloud 면
    무효 픽셀이 NaN 으로 남아 있으므로 반드시 걸러야 한다.
    """
    values = np.asarray(points, dtype=np.float64)
    if values.size == 0:
        return values.reshape(0, 3)
    return values[np.isfinite(values).all(axis=1)]


def limit_range(points, min_range, max_range):
    """센서 원점 기준 수평거리로 유효 구간만 남긴다.

    D455 의 depth Z-min(약 0.4 m) 안쪽은 값이 나와도 신뢰할 수 없고, 원거리는
    셀당 픽셀 수가 한 자릿수로 떨어져(6 m 에서 약 8 px) 노이즈 한두 점이 그대로
    점유가 된다. 두 끝을 모두 잘라야 한다.
    """
    if points.size == 0:
        return points
    distance = np.hypot(points[:, 0], points[:, 1])
    keep = (distance >= float(min_range)) & (distance <= float(max_range))
    return points[keep]


def limit_range_from(points, origin_xy, max_range):
    """주어진 원점 기준 수평거리 상한만 거른다.

    limit_range 는 height_frame 원점(= base_footprint, 지면 중심) 기준이다.
    카메라는 그 원점에서 x 로 0.114 m 떨어져 있으므로(URDF base_link ->
    camera_link 0.11398), "카메라 기준 전방 N m" 를 원하면 원점을 옮겨 재야
    한다. origin_xy 는 height_frame 안의 카메라 위치다.

    왜 밴드에만 따로 상한이 필요한가 — 노이즈가 거리의 제곱으로 붙는다:

        거리   셀 폭   셀 면적   깊이오차(2 %)   오차/셀
        1.5 m  14.0 px   197 px²    0.030 m       0.6
        2.5 m   8.4 px    71 px²    0.050 m       1.0   <- 오차 = 1 셀
        6.0 m   3.5 px    12 px²    0.120 m       2.4

    6 m 에서는 셀 하나를 덮는 픽셀이 12 개뿐이라 노이즈 한두 점이 그대로 점유가
    되고, 깊이오차만으로도 점이 2.4 셀 퍼진다. 2D 격자는 그 점 하나로 셀을
    BLOCKED 으로 세우므로(프레임 임계 depth_hit_threshold 만 넘으면 된다)
    원거리 점을 넣을 이유가 없다.

    3D 쪽(cloud_filtered -> octomap)은 log-odds 로 여러 관측을 누적해 노이즈를
    걸러내므로 같은 상한을 걸지 않는다. 그래서 이 함수는 밴드에만 쓴다.
    """
    if points.size == 0:
        return points
    origin = np.asarray(origin_xy, dtype=np.float64).reshape(2)
    distance = np.hypot(points[:, 0] - origin[0], points[:, 1] - origin[1])
    return points[distance <= float(max_range)]


def drop_self_observations(points, box_min, box_max):
    """차체 자기 관측을 제거한다.

    box_min/box_max 는 height_frame 기준 축정렬 상자다. 설계문서 §3.2 는 "실제
    외부 장애물을 지우지 않도록" 요구하므로 상자를 차체보다 크게 잡지 않는다.
    로봇팔·적재물이 붙으면 그 형상까지 포함하도록 넓혀야 한다(1단계에는 없음).
    """
    if points.size == 0:
        return points
    inside = np.all(
        (points >= np.asarray(box_min, dtype=np.float64))
        & (points <= np.asarray(box_max, dtype=np.float64)),
        axis=1,
    )
    return points[~inside]


def select_height_band(points, z_min, z_max):
    """수직 z 가 [z_min, z_max] 인 점만 남긴다.

    이 밴드가 설계문서 §4.1 의 투영 구간이다. 바닥면을 밴드에서 빼는 것도,
    로봇보다 높은 구조물을 빼는 것도 여기서 일어난다.
    """
    if points.size == 0:
        return points
    z = points[:, 2]
    return points[(z >= float(z_min)) & (z <= float(z_max))]


def voxel_downsample(points, leaf):
    """Leaf 크기 격자마다 점 하나만 남긴다.

    거리에 따라 셀당 픽셀 수가 1137(0.5 m)에서 8(6 m)까지 변하므로, 다운샘플을
    안 하면 근거리 점이 연산량을 독차지한다. 격자 인덱스로 중복을 지우는
    방식이라 정렬 비용이 없다.
    """
    if points.size == 0 or leaf <= 0.0:
        return points
    keys = np.floor(points / float(leaf)).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def to_flat_cells(points, width, height, resolution, origin_x, origin_y):
    """Map frame 의 (x, y) 를 OccupancyGrid flat 인덱스로 바꾼다.

    격자 밖의 점은 버린다. 같은 셀에 여러 점이 떨어져도 한 번만 센다.
    """
    if points.size == 0:
        return np.empty(0, dtype=np.int64)
    cell_x = np.floor((points[:, 0] - origin_x) / resolution).astype(np.int64)
    # OccupancyGrid 는 row 0 이 최소 y 다. 이미지처럼 뒤집지 않는다.
    cell_y = np.floor((points[:, 1] - origin_y) / resolution).astype(np.int64)
    valid = (
        (cell_x >= 0) & (cell_x < width) & (cell_y >= 0) & (cell_y < height)
    )
    return np.unique(cell_y[valid] * width + cell_x[valid])


def transform_points(points, translation, rotation_matrix):
    """3-D 강체 변환을 건다. rotation_matrix 는 3x3."""
    if points.size == 0:
        return points
    return points @ np.asarray(rotation_matrix, dtype=np.float64).T + np.asarray(
        translation, dtype=np.float64
    )


def quaternion_to_matrix(x, y, z, w):
    """단위 quaternion 을 3x3 회전행렬로.

    TransformStamped 의 회전을 그대로 받는다. tf2 의 do_transform_cloud 는
    PointCloud2 를 통째로 다루지만, 여기서는 numpy 배열만 다루므로 행렬이 필요하다.
    """
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError('quaternion must not be zero')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
