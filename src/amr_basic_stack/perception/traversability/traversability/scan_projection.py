"""LaserScan 을 map frame 의 셀 인덱스로 투영한다.

여기도 ROS 메시지에 의존하지 않는다 — 배열과 스칼라만 받는다.
반환 인덱스는 OccupancyGrid 순서(row 0 = 최소 y)다.
global_planner 의 같은 이름 함수는 이미지 순서를 쓰므로 y 가 뒤집혀 있다.
"""

import numpy as np


def laser_endpoints(ranges, angle_min, angle_increment, minimum, maximum):
    """유효 range 만 골라 센서 frame 의 (x, y) 끝점으로 바꾼다."""
    values = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + np.arange(values.size, dtype=np.float64) * angle_increment
    valid = np.isfinite(values) & (values >= minimum) & (values <= maximum)
    return np.column_stack(
        (
            values[valid] * np.cos(angles[valid]),
            values[valid] * np.sin(angles[valid]),
        )
    )


def transform_points(points, translation_x, translation_y, yaw):
    """센서 frame 의 점들에 2-D 강체 변환을 건다."""
    if points.size == 0:
        return points
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    return np.column_stack(
        (
            translation_x + cos_yaw * points[:, 0] - sin_yaw * points[:, 1],
            translation_y + sin_yaw * points[:, 0] + cos_yaw * points[:, 1],
        )
    )


def world_to_flat_cells(world_points, width, height, resolution, origin_x, origin_y):
    """Map frame 좌표를 OccupancyGrid 의 flat 인덱스로 바꾼다.

    격자 밖의 점은 버린다. 한 스캔에서 같은 셀이 여러 번 맞아도 한 번만 센다.
    """
    if world_points.size == 0:
        return np.empty(0, dtype=np.int64)
    cell_x = np.floor((world_points[:, 0] - origin_x) / resolution).astype(np.int64)
    # OccupancyGrid 는 row 0 이 최소 y 다. 이미지처럼 뒤집지 않는다.
    cell_y = np.floor((world_points[:, 1] - origin_y) / resolution).astype(np.int64)
    valid = (
        (cell_x >= 0) & (cell_x < width) & (cell_y >= 0) & (cell_y < height)
    )
    return np.unique(cell_y[valid] * width + cell_x[valid])


def accumulated_hit_mask(cell_batches, shape, minimum_hits):
    """서로 다른 스캔 minimum_hits 개 이상에서 관측된 셀만 True 로 만든다."""
    counts = np.zeros(shape[0] * shape[1], dtype=np.uint16)
    for cells in cell_batches:
        np.add.at(counts, cells, 1)
    return counts.reshape(shape) >= int(minimum_hits)


def raycast_free_cells(origin, world_points, width, height, resolution,
                       origin_x, origin_y, step_ratio=0.5, stop_margin=1.0):
    """센서 원점에서 각 끝점까지의 광선이 관통한 셀을 돌려준다 (끝점 제외).

    이것이 설계문서 §6 의 "삭제에는 해당 공간을 관측한 증거가 필요하다" 를
    만드는 연산이다. 빔이 지나갔다는 것은 그 평면의 그 셀이 비어 있었다는
    증거이며, 기준 맵의 오래된 점유를 해제할 근거가 된다.

    끝점 셀은 장애물 자신이므로 항상 남기고, 그 **앞 stop_margin 셀**도
    추가로 남긴다. stop_margin=0 이면 끝점 셀만 제외한다. 측정 오차가
    한 셀 정도 있을 때 장애물 바로 앞을 뚫어버리는 것을 막기 위한 여유다.

    step_ratio 는 광선 위 표본 간격을 resolution 의 배수로 준다. 값이 크면
    셀을 건너뛰어 해제가 성기게 되는데, 해제를 덜 하는 쪽은 안전한 방향이다.
    """
    origin = np.asarray(origin, dtype=np.float64)
    points = np.asarray(world_points, dtype=np.float64)
    if points.size == 0:
        return np.empty(0, dtype=np.int64)

    # 끝점 셀 하나(+1) 에 여유 stop_margin 셀을 더한 만큼을 남긴다.
    keep_back = float(resolution) * (1.0 + float(stop_margin))

    delta = points - origin
    distance = np.hypot(delta[:, 0], delta[:, 1])
    reachable = distance > keep_back
    if not np.any(reachable):
        return np.empty(0, dtype=np.int64)
    delta = delta[reachable]
    distance = distance[reachable]

    step = max(float(resolution) * float(step_ratio), 1.0e-6)
    sample_count = int(np.ceil(distance.max() / step))
    if sample_count <= 0:
        return np.empty(0, dtype=np.int64)

    offsets = (np.arange(sample_count, dtype=np.float64) * step)[None, :]
    unit = delta / distance[:, None]
    xs = origin[0] + unit[:, 0:1] * offsets
    ys = origin[1] + unit[:, 1:2] * offsets

    cell_x = np.floor((xs - origin_x) / resolution).astype(np.int64)
    cell_y = np.floor((ys - origin_y) / resolution).astype(np.int64)
    inside = (
        (offsets < (distance[:, None] - keep_back))
        & (cell_x >= 0) & (cell_x < width)
        & (cell_y >= 0) & (cell_y < height)
    )
    return np.unique(cell_y[inside] * width + cell_x[inside])
