"""OccupancyGrid 기반 STATIC 트랙 출력 필터.

ROS 메시지와 TF 처리는 노드에 남기고, 여기서는 두 가지만 한다.

* 점유 셀로부터의 Euclidean distance field를 만든다.
* STATIC 트랙이 벽 근처에 연속해서 머물 때만 출력 억제 상태로 바꾼다.

UNKNOWN/DYNAMIC 트랙은 이 필터가 절대 억제하지 않는다. 벽을 따라 걷는 사람을
정적 지도 하나만 보고 지우지 않기 위한 안전 규칙이다.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


@dataclass
class GridFilterConfig:
    """노드가 그대로 ROS parameter로 노출하는 grid filter 설정."""

    # Semantic objects must remain obstacles even when stationary near a wall.
    enabled: bool = False
    map_topic: str = '/map'
    occupied_threshold: int = 65
    enter_distance: float = 0.15
    exit_distance: float = 0.20
    confirm_frames: int = 3

    @classmethod
    def field_defaults(cls):
        return {field.name: getattr(cls, field.name, field.default)
                for field in cls.__dataclass_fields__.values()}

    def validate(self):
        if not self.map_topic:
            raise ValueError('grid_filter_map_topic must not be empty')
        if not 0 <= self.occupied_threshold <= 100:
            raise ValueError(
                'grid_filter_occupied_threshold must be in [0,100]')
        if self.enter_distance < 0.0:
            raise ValueError('grid_filter_enter_distance must be non-negative')
        if self.exit_distance < self.enter_distance:
            raise ValueError(
                'grid_filter_exit_distance must be >= enter_distance')
        if self.confirm_frames < 1:
            raise ValueError('grid_filter_confirm_frames must be >= 1')


class OccupancyDistanceGrid:
    """점유 셀까지의 거리를 미터 단위로 조회하는 회전 가능 격자."""

    def __init__(self):
        self.frame_id = ''
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0
        self._distance = None

    @property
    def ready(self):
        return self._distance is not None

    def update(self, data, width, height, resolution, origin_x, origin_y,
               origin_yaw, frame_id, occupied_threshold):
        """OccupancyGrid wire 배열로 distance field를 다시 만든다.

        unknown(-1)은 벽으로 보지 않는다. 지도 밖/미확인은 억제 근거가 아니므로
        조회 시 무한 거리로 취급해 fail-open 한다.
        """
        width, height = int(width), int(height)
        resolution = float(resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError('grid geometry must be positive')
        values = np.asarray(data, dtype=np.int16)
        if values.size != width * height:
            raise ValueError(
                f'grid data size {values.size} != {width}x{height}')

        occupied = values.reshape(height, width) >= int(occupied_threshold)
        if np.any(occupied):
            distance = distance_transform_edt(~occupied) * resolution
        else:
            distance = np.full((height, width), np.inf, dtype=np.float64)

        self.frame_id = str(frame_id)
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.origin_yaw = float(origin_yaw)
        self._distance = distance

    def distances(self, points):
        """map header frame의 Nx2 점마다 최근접 점유 셀 거리[m]를 반환."""
        points = np.asarray(points, dtype=np.float64)
        if points.size == 0:
            return np.empty(0, dtype=np.float64)
        points = points.reshape(-1, 2)
        result = np.full(points.shape[0], np.inf, dtype=np.float64)
        if not self.ready:
            return result

        # OccupancyGrid origin pose의 역변환. origin yaw가 0이 아닌 지도도
        # cell 좌표를 정확히 찾는다.
        dx = points[:, 0] - self.origin_x
        dy = points[:, 1] - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        local_x = c * dx + s * dy
        local_y = -s * dx + c * dy
        cell_x = np.floor(local_x / self.resolution).astype(np.int64)
        cell_y = np.floor(local_y / self.resolution).astype(np.int64)
        inside = (
            (cell_x >= 0) & (cell_x < self.width)
            & (cell_y >= 0) & (cell_y < self.height)
        )
        result[inside] = self._distance[cell_y[inside], cell_x[inside]]
        return result


class StaticTrackSuppressor:
    """트랙별 진입 카운터와 거리 hysteresis를 관리한다."""

    def __init__(self, config):
        self.config = config
        self._near_counts = {}
        self._suppressed = set()

    def reset(self):
        self._near_counts.clear()
        self._suppressed.clear()

    def select(self, tracks, distances, static_value):
        """(유지할 트랙, 억제한 트랙)을 반환한다.

        distances는 tracks와 순서가 같아야 한다. STATIC이 아닌 트랙은 이전
        억제 상태가 있더라도 즉시 해제한다.
        """
        if len(tracks) != len(distances):
            raise ValueError('tracks and distances must have equal length')

        live_ids = {int(track.id) for track in tracks}
        for track_id in list(self._near_counts):
            if track_id not in live_ids:
                self._near_counts.pop(track_id, None)
        self._suppressed.intersection_update(live_ids)

        kept, suppressed = [], []
        cfg = self.config
        for track, distance in zip(tracks, distances):
            track_id = int(track.id)
            if track.motion != static_value:
                self._near_counts.pop(track_id, None)
                self._suppressed.discard(track_id)
                kept.append(track)
                continue

            if track_id in self._suppressed:
                if distance <= cfg.exit_distance:
                    suppressed.append(track)
                else:
                    self._suppressed.remove(track_id)
                    self._near_counts.pop(track_id, None)
                    kept.append(track)
                continue

            if distance <= cfg.enter_distance:
                count = self._near_counts.get(track_id, 0) + 1
                self._near_counts[track_id] = count
                if count >= cfg.confirm_frames:
                    self._suppressed.add(track_id)
                    suppressed.append(track)
                else:
                    kept.append(track)
            else:
                self._near_counts.pop(track_id, None)
                kept.append(track)
        return kept, suppressed
