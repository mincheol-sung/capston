"""AMCL 수렴 판정. ROS 에 의존하지 않는다.

전역 측위(global localization)를 걸면 AMCL 은 자유공간 전체에 파티클을 뿌린다.
그 구름이 한 점으로 모였는지를 판정해서, 모였을 때만 그 pose 를 cartographer 의
초기 위치로 넘긴다.

판정에 두 가지를 같이 본다.

1. **공분산** — x, y, yaw 표준편차가 임계값 아래.
2. **이동 거리** — 그 전에 로봇이 최소 거리 이상 움직였을 것.

두 번째가 필요한 이유: AMCL 은 로봇이 움직여야(update_min_d/update_min_a)
파티클을 갱신한다. 가만히 있으면 초기 구름이 그대로 남아 있는데도 공분산이
줄어드는 구간이 생길 수 있고, 무엇보다 대칭적인 복도에서는 한 자리에서 본
스캔만으로 엉뚱한 곳에 자신 있게 수렴한다. 서로 다른 지점에서 본 관측이
쌓여야 그 모호성이 풀린다.
"""

import math


def pose_standard_deviations(covariance):
    """공분산 6x6 row-major 에서 (x, y, yaw) 표준편차를 뽑는다.

    인덱스는 ROS 규약을 따른다 — 대각선의 0, 7, 35 가 각각 xx, yy, yaw-yaw.
    """
    values = list(covariance)
    if len(values) != 36:
        raise ValueError(f'covariance must have 36 elements, got {len(values)}')
    return (
        math.sqrt(max(values[0], 0.0)),
        math.sqrt(max(values[7], 0.0)),
        math.sqrt(max(values[35], 0.0)),
    )


class ConvergenceTracker:
    """연속 N 회 기준을 만족해야 수렴으로 본다.

    한 번 튄 값으로 성급히 확정하지 않기 위해서다. 기준을 벗어나면 카운터를
    0 으로 되돌린다.
    """

    def __init__(self, position_std, yaw_std, required_samples, min_travel):
        if position_std <= 0.0 or yaw_std <= 0.0:
            raise ValueError('std thresholds must be positive')
        if required_samples < 1:
            raise ValueError('required_samples must be at least 1')
        if min_travel < 0.0:
            raise ValueError('min_travel must not be negative')
        self.position_std = float(position_std)
        self.yaw_std = float(yaw_std)
        self.required_samples = int(required_samples)
        self.min_travel = float(min_travel)
        self.travel = 0.0
        self.streak = 0
        self.last_std = None

    def reset(self):
        self.travel = 0.0
        self.streak = 0
        self.last_std = None

    def add_travel(self, distance):
        """이동 거리를 누적한다. 음수는 무시한다."""
        if distance > 0.0:
            self.travel += float(distance)

    @property
    def travel_satisfied(self):
        return self.travel >= self.min_travel

    def update(self, covariance):
        """새 /amcl_pose 하나를 반영하고, 지금 수렴 조건을 만족하는지 돌려준다."""
        std_x, std_y, std_yaw = pose_standard_deviations(covariance)
        self.last_std = (std_x, std_y, std_yaw)

        tight = (
            std_x <= self.position_std
            and std_y <= self.position_std
            and std_yaw <= self.yaw_std
        )
        self.streak = self.streak + 1 if tight else 0
        return self.converged

    @property
    def converged(self):
        return self.travel_satisfied and self.streak >= self.required_samples

    def status(self):
        """사람이 읽을 진행 상황 한 줄."""
        if self.last_std is None:
            return f'아직 /amcl_pose 없음 (이동 {self.travel:.2f} m)'
        std_x, std_y, std_yaw = self.last_std
        return (
            f'std x={std_x:.3f} y={std_y:.3f} m yaw={math.degrees(std_yaw):.1f} deg '
            f'(기준 {self.position_std:.3f} m / '
            f'{math.degrees(self.yaw_std):.1f} deg) '
            f'| 연속 {self.streak}/{self.required_samples} '
            f'| 이동 {self.travel:.2f}/{self.min_travel:.2f} m'
        )


class MapMatcher:
    """후보 pose 가 실제로 맞는지 스캔-지도 정합으로 검증한다.

    공분산만으로는 부족하다. AMCL 은 대칭적인 실내에서 **자신 있게 틀린 곳**에
    수렴한다. 2026-09-09 실측: test_imu_scan 에서 위치 오차 6.4 m, 방향 165 도
    (전형적인 180 도 모호성)인데 std 는 0.215 m / 8.2 도 였다.

    그래서 수렴한 pose 에 현재 스캔을 얹어 보고, 끝점이 지도의 점유 셀에
    실제로 붙는지를 센다. 180 도 뒤집힌 pose 는 벽 모양이 안 맞으므로 점수가
    크게 떨어진다.

    거리장은 점유 셀까지의 유클리드 거리다(미터). 미관측 셀은 점유가 아니므로
    그 안에서 끝난 광선은 자동으로 miss 가 된다.
    """

    def __init__(self, occupancy, width, height, resolution,
                 origin_x, origin_y, occupied_threshold=65,
                 match_distance=0.20):
        import numpy as np
        from scipy.ndimage import distance_transform_edt

        grid = np.asarray(occupancy, dtype=np.int16).reshape(height, width)
        occupied = grid >= int(occupied_threshold)
        if not occupied.any():
            raise ValueError('지도에 점유 셀이 하나도 없다')
        self._np = np
        self.distance = distance_transform_edt(~occupied) * float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.match_distance = float(match_distance)

    def score(self, points_in_map):
        """끝점들이 점유 셀에 붙은 비율과 표본 수를 돌려준다 (map 좌표).

        반환: (score, used) — score 는 0~1, used 는 격자 안에 든 점 수.
        used 가 적으면 점수를 믿으면 안 된다.
        """
        np = self._np
        points = np.asarray(points_in_map, dtype=np.float64).reshape(-1, 2)
        if points.shape[0] == 0:
            return 0.0, 0
        cell_x = np.floor((points[:, 0] - self.origin_x) / self.resolution)
        cell_y = np.floor((points[:, 1] - self.origin_y) / self.resolution)
        cell_x = cell_x.astype(np.int64)
        cell_y = cell_y.astype(np.int64)
        inside = (
            (cell_x >= 0) & (cell_x < self.width)
            & (cell_y >= 0) & (cell_y < self.height)
        )
        used = int(inside.sum())
        if used == 0:
            return 0.0, 0
        near = self.distance[cell_y[inside], cell_x[inside]] <= self.match_distance
        return float(near.sum()) / used, used
