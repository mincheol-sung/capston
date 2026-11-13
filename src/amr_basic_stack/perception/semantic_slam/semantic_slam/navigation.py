"""Object query and approach policies independent of ROS callbacks."""

import math

import numpy as np


def select_object(
    objects, object_id='', class_name='', alias='', near_object_id='', near_distance=2.0
):
    """Resolve an explicit query; similar chairs need an ID or unique alias."""
    if not any((object_id, class_name, alias)):
        raise ValueError('Specify object_id, class_name, or alias')
    records = list(objects)
    if near_object_id:
        reference = next((r for r in records if r.object_id == near_object_id), None)
        if reference is None:
            raise LookupError('Near-reference object is unknown')
    result = []
    for obj in records:
        if obj.status not in (1, 2):
            continue
        if object_id and obj.object_id != object_id:
            continue
        if class_name and obj.class_name != class_name:
            continue
        if alias and obj.alias != alias:
            continue
        if near_object_id:
            distance = math.hypot(
                obj.position.x - reference.position.x, obj.position.y - reference.position.y
            )
            if obj.object_id == near_object_id or distance > near_distance:
                continue
        result.append(obj)
    if len(result) > 1:
        raise ValueError('Ambiguous object query; use object_id or a unique alias')
    if not result:
        raise LookupError('No confirmed object matches')
    return result[0]


def approach_candidates(center, extent, standoff=0.8, count=16):
    """Sample outside an observed AABB, facing its center."""
    center, extent = np.asarray(center, dtype=float), np.asarray(extent, dtype=float)
    if not np.isfinite(np.r_[center, extent, standoff]).all() or np.any(extent < 0):
        raise ValueError('Invalid object geometry')
    result = []
    for angle in np.linspace(-math.pi, math.pi, count, endpoint=False):
        direction = np.array([math.cos(angle), math.sin(angle)])
        radii = [extent[i] / 2 / abs(direction[i]) for i in range(2) if abs(direction[i]) > 1e-9]
        xy = center[:2] + direction * (min(radii) + standoff)
        yaw = math.atan2(center[1] - xy[1], center[0] - xy[0])
        result.append((float(xy[0]), float(xy[1]), yaw))
    return result


def at_approach(
    pose, goal, linear_speed, angular_speed, position_tolerance=0.15, yaw_tolerance=0.25
):
    """Arrival needs position, facing direction, and a stopped base."""
    if not np.isfinite([*pose, *goal, linear_speed, angular_speed]).all():
        return False
    error = math.atan2(math.sin(pose[2] - goal[2]), math.cos(pose[2] - goal[2]))
    return (
        math.dist(pose[:2], goal[:2]) <= position_tolerance
        and abs(error) <= yaw_tolerance
        and abs(linear_speed) < 0.05
        and abs(angular_speed) < 0.1
    )
