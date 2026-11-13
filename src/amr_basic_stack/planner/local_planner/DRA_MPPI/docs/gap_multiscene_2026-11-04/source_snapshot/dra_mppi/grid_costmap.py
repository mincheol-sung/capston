"""Distance-field representation of a ROS occupancy grid."""

from dataclasses import dataclass

import numpy as np
from scipy.ndimage import distance_transform_edt

from .geometry import transform_points


@dataclass(frozen=True)
class GridMetadata:
    """Geometry needed to convert world coordinates into grid cells."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float


class DistanceField:
    """Immutable clearance lookup generated from an occupancy grid."""

    def __init__(self, metadata, clearance, frame_id='map'):
        self.metadata = metadata
        self.clearance = np.array(clearance, dtype=np.float64, copy=True)
        self.clearance.setflags(write=False)
        self.frame_id = frame_id
        expected = (metadata.height, metadata.width)
        if self.clearance.shape != expected:
            raise ValueError(f'clearance shape must be {expected}')

    @classmethod
    def from_occupancy(
        cls,
        data,
        metadata,
        frame_id='map',
        occupied_threshold=65,
        unknown_is_occupied=True,
    ):
        """Construct a metric distance field from flattened occupancy data."""
        grid = np.asarray(data, dtype=np.int16).reshape(
            metadata.height,
            metadata.width,
        )
        if (metadata.resolution <= 0 or metadata.width < 1 or metadata.height < 1
                or not np.all(np.isfinite([
                    metadata.resolution, metadata.origin_x,
                    metadata.origin_y, metadata.origin_yaw]))):
            raise ValueError('invalid grid geometry')
        occupied = grid >= occupied_threshold
        if unknown_is_occupied:
            occupied |= grid < 0
        clearance = (distance_transform_edt(~occupied) * metadata.resolution
                     if np.any(occupied) else np.full(grid.shape, np.inf))
        return cls(metadata, clearance, frame_id=frame_id)

    def trajectory_collisions(self, positions, initial_xy, robot_radius,
                              margin=0.0, source_to_grid=None):
        """Conservative swept-disc check including the initial robot footprint.

        Interpolate each segment at <= half-cell spacing. EDT is measured
        between cell centres, so subtract a cell diagonal for the query and
        obstacle-cell offsets, plus half a sample interval for the sweep.
        Also check distance to the map boundary (not only centre inclusion).
        """
        points = np.asarray(positions, dtype=float)
        if points.ndim != 3 or points.shape[-1] != 2:
            raise ValueError('positions must have shape (K,T,2)')
        if robot_radius < 0 or margin < 0:
            raise ValueError('footprint radius and margin must be nonnegative')
        if not points.shape[0]:
            return np.empty(0, dtype=bool)
        initial = np.broadcast_to(initial_xy, (points.shape[0], 1, 2))
        path = np.concatenate((initial, points), axis=1)
        if not np.all(np.isfinite(path)):
            raise ValueError('nonfinite collision query')
        if source_to_grid is not None:
            path = transform_points(path, source_to_grid)
        spacing = self.metadata.resolution * 0.5
        starts, delta = path[:, :-1], np.diff(path, axis=1)
        subdivisions = max(1, int(np.ceil(
            np.max(np.linalg.norm(delta, axis=-1), initial=0.0) / spacing)))
        fractions = np.linspace(0.0, 1.0, subdivisions + 1)
        queries = (starts[:, :, None, :]
                   + delta[:, :, None, :] * fractions[None, None, :, None])
        clearance = self.query(queries)
        meta = self.metadata
        dx, dy = queries[..., 0] - meta.origin_x, queries[..., 1] - meta.origin_y
        x = np.cos(meta.origin_yaw) * dx + np.sin(meta.origin_yaw) * dy
        y = -np.sin(meta.origin_yaw) * dx + np.cos(meta.origin_yaw) * dy
        boundary = np.minimum.reduce((
            x, y, meta.width * meta.resolution - x,
            meta.height * meta.resolution - y,
        ))
        radius = robot_radius + margin
        occupied = clearance <= radius + np.sqrt(2.0) * meta.resolution + spacing / 2
        outside = boundary <= radius + spacing / 2
        return np.any(occupied | outside, axis=(1, 2))

    def trajectory_clearances(self, positions, initial_xy, source_to_grid=None):
        """Return minimum map clearance along every rollout segment.

        The result has shape ``(K, T)``.  Entry ``[k, t]`` is the minimum
        distance-field value swept from state ``t-1`` to state ``t`` (including
        the measured initial position for ``t=0``).  Sampling at no more than
        half a grid cell prevents a rollout step from jumping over a thin
        occupied region. Grid-diagonal and half-sample padding give a
        conservative centre-clearance lower bound, including the map boundary.
        The caller subtracts the physical robot radius exactly once.
        """
        points = np.asarray(positions, dtype=np.float64)
        if points.ndim != 3 or points.shape[-1] != 2:
            raise ValueError('positions must have shape (K,T,2)')
        if not points.shape[0]:
            return np.empty(points.shape[:2], dtype=np.float64)
        initial = np.broadcast_to(initial_xy, (points.shape[0], 1, 2))
        path = np.concatenate((initial, points), axis=1)
        if not np.all(np.isfinite(path)):
            raise ValueError('nonfinite clearance query')
        if source_to_grid is not None:
            path = transform_points(path, source_to_grid)

        starts = path[:, :-1]
        delta = np.diff(path, axis=1)
        spacing = self.metadata.resolution * 0.5
        subdivisions = max(1, int(np.ceil(
            np.max(np.linalg.norm(delta, axis=-1), initial=0.0) / spacing
        )))
        fractions = np.linspace(0.0, 1.0, subdivisions + 1)
        queries = (
            starts[:, :, None, :]
            + delta[:, :, None, :] * fractions[None, None, :, None]
        )
        meta = self.metadata
        dx = queries[..., 0]-meta.origin_x
        dy = queries[..., 1]-meta.origin_y
        x = np.cos(meta.origin_yaw)*dx+np.sin(meta.origin_yaw)*dy
        y = -np.sin(meta.origin_yaw)*dx+np.cos(meta.origin_yaw)*dy
        boundary = np.minimum.reduce((
            x, y, meta.width*meta.resolution-x, meta.height*meta.resolution-y))
        clearance = np.minimum(
            self.query(queries)-np.sqrt(2.)*meta.resolution, boundary)-spacing/2.
        return np.maximum(0., np.min(clearance, axis=2))

    def state_clearances(self, positions, source_to_grid=None):
        """Conservative centre clearance at each future state, in metres."""
        points = np.asarray(positions, dtype=float)
        if source_to_grid is not None:
            points = transform_points(points, source_to_grid)
        meta = self.metadata
        dx, dy = points[..., 0]-meta.origin_x, points[..., 1]-meta.origin_y
        x = np.cos(meta.origin_yaw)*dx+np.sin(meta.origin_yaw)*dy
        y = -np.sin(meta.origin_yaw)*dx+np.cos(meta.origin_yaw)*dy
        boundary = np.minimum.reduce((x, y, meta.width*meta.resolution-x,
                                      meta.height*meta.resolution-y))
        return np.maximum(0., np.minimum(self.query(points)-np.sqrt(2.)*meta.resolution,
                                        boundary))

    def query(self, points, source_to_grid=None):
        """Return clearance in metres for points, with outside cells set to zero."""
        values = np.asarray(points, dtype=np.float64)
        if source_to_grid is not None:
            values = transform_points(values, source_to_grid)

        meta = self.metadata
        dx = values[..., 0] - meta.origin_x
        dy = values[..., 1] - meta.origin_y
        cos_yaw = np.cos(meta.origin_yaw)
        sin_yaw = np.sin(meta.origin_yaw)
        local_x = cos_yaw * dx + sin_yaw * dy
        local_y = -sin_yaw * dx + cos_yaw * dy
        cell_x = np.floor(local_x / meta.resolution).astype(np.int64)
        cell_y = np.floor(local_y / meta.resolution).astype(np.int64)

        valid = (
            (cell_x >= 0)
            & (cell_x < meta.width)
            & (cell_y >= 0)
            & (cell_y < meta.height)
        )
        result = np.zeros(values.shape[:-1], dtype=np.float64)
        result[valid] = self.clearance[cell_y[valid], cell_x[valid]]
        return result
