"""Timestamp anchors into optimized Cartographer tracking-frame node poses."""

from bisect import bisect_left
from dataclasses import dataclass
import math

import numpy as np


STAMP_SNAP_TOLERANCE = 1e-7


class FrozenPoints(tuple):
    """Immutable raw evidence with a reusable, read-only float64 array view."""

    def __new__(cls, points):
        if isinstance(points, cls):
            return points
        array = np.asarray(tuple(points), dtype='<f8')
        if (array.ndim != 2 or array.shape[1] != 3 or not len(array)
                or not np.isfinite(array).all()):
            raise ValueError('Invalid raw anchor points')
        result = super().__new__(cls, (tuple(row) for row in array.tolist()))
        # Immutable bytes prevent callers from making the shared view writeable.
        result._array = np.frombuffer(array.tobytes(), dtype='<f8').reshape(-1, 3)
        return result

    def __array__(self, dtype=None, copy=None):
        """Expose unchanged numeric evidence without converting Python rows again."""
        array = self._array if dtype is None else self._array.astype(dtype, copy=False)
        return array.copy() if copy else array

    def __eq__(self, other):
        """Keep value comparisons compatible with legacy JSON point lists."""
        try:
            return np.array_equal(self._array, np.asarray(other))
        except (ValueError, TypeError):
            return False

    def __ne__(self, other):
        """Use the same legacy-compatible value comparison for inequality."""
        return not self == other

    __hash__ = None


@dataclass
class NodePose:
    """A pose-graph node identified by trajectory ID and immutable ROS stamp."""

    stamp: float
    translation: list
    quaternion: list


def quaternion_matrix(quaternion):
    """Validate and convert xyzw to a rotation matrix."""
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid graph quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def interpolate_pose(first, second, weight):
    """Linear translation and shortest-arc quaternion SLERP, never extrapolate."""
    if not 0 <= weight <= 1:
        raise ValueError('Invalid interpolation weight')
    a = np.asarray(first.quaternion, dtype=float)
    b = np.asarray(second.quaternion, dtype=float)
    quaternion_matrix(a)
    quaternion_matrix(b)
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    dot = float(a @ b)
    if dot < 0:
        b, dot = -b, -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        q = (1 - weight) * a + weight * b
    else:
        angle = math.acos(dot)
        q = (math.sin((1 - weight) * angle) * a + math.sin(weight * angle) * b)
        q /= math.sin(angle)
    translation = ((1 - weight) * np.asarray(first.translation)
                   + weight * np.asarray(second.translation))
    return quaternion_matrix(q), translation


class PoseGraph:
    """Authoritative snapshots; removed nodes never silently reuse old poses."""

    def __init__(self, graph_id, max_gap=1.0, max_nodes=20000):
        if not graph_id.strip() or max_gap <= 0 or max_nodes < 2:
            raise ValueError('A graph session ID and positive graph limits are required')
        self.graph_id, self.max_gap, self.max_nodes = graph_id, max_gap, max_nodes
        self.nodes = {}
        self.stamps = {}
        # (first, last) node stamps of the snapshot each trajectory replaced, so a
        # missing prefix can be told apart from a discontinuous (restarted) one.
        self.previous = {}
        # First node stamp after each trajectory's latest discontinuity; frozen
        # evidence from before it belongs to another graph and is void.
        self.epoch = {}
        self.active = None

    def continue_from(self, previous):
        """Inherit trimming continuity from the snapshot this graph replaces."""
        self.previous = {k: (s[0], s[-1]) for k, s in previous.stamps.items() if s}
        self.epoch = dict(previous.epoch)

    def replace(self, trajectory_id, nodes):
        """Validate an entire service response before replacing its snapshot."""
        if trajectory_id < 0 or len(nodes) > self.max_nodes:
            raise ValueError('Invalid trajectory or graph capacity exceeded')
        # Validate the whole response before committing, without constructing a
        # rotation matrix for every node. Only referenced anchors need rotations.
        if nodes:
            stamps_array = np.asarray([node.stamp for node in nodes])
            if stamps_array.dtype.kind not in 'biuf':
                if not all(math.isfinite(node.stamp) for node in nodes):
                    raise ValueError('Invalid graph stamp')
                stamps_array = stamps_array.astype(float)
            positions = np.asarray([node.translation for node in nodes], dtype=float)
            quaternions = np.asarray([node.quaternion for node in nodes], dtype=float)
            if (positions.shape != (len(nodes), 3)
                    or quaternions.shape != (len(nodes), 4)
                    or not np.isfinite(stamps_array).all() or np.any(stamps_array <= 0)
                    or not np.isfinite(positions).all()
                    or not np.isfinite(quaternions).all()
                    or np.any(np.linalg.norm(quaternions, axis=1) < 1e-9)):
                raise ValueError('Invalid/duplicate graph node')
        snapshot = {node.stamp: node for node in nodes}
        if len(snapshot) != len(nodes):
            raise ValueError('Invalid/duplicate graph node')
        old = self.stamps.get(trajectory_id)
        if old:
            self.previous[trajectory_id] = (old[0], old[-1])
        stamps = sorted(snapshot)
        previous = self.previous.get(trajectory_id)
        if stamps and previous and stamps[0] > previous[1]:
            self.epoch[trajectory_id] = stamps[0]
        self.nodes[trajectory_id] = snapshot
        self.stamps[trajectory_id] = stamps

    def trimmed(self, anchor):
        """
        Check whether the anchor's nodes were trimmed from a continuous trajectory.

        Cartographer (e.g. pure_localization_trimmer) never re-optimizes trimmed nodes,
        so the anchor's last resolved pose is final rather than invalid. The anchor's
        left node must have been inside the previous snapshot, and the new snapshot
        must overlap it. A gap in the middle, a deleted trajectory, or a restart
        (even when polled again later) is not trimming.
        """
        stamps = self.stamps.get(anchor['trajectory_id'])
        previous = self.previous.get(anchor['trajectory_id'])
        return (bool(stamps) and previous is not None and 'pose' in anchor
                and previous[0] <= anchor['left'] < stamps[0] <= previous[1])

    def bind(self, stamp, points, covariance):
        """Anchor capture-time tracking-frame data between two graph nodes."""
        pts = np.asarray(points, dtype=float)
        cov = np.asarray(covariance, dtype=float).reshape(3, 3)
        if (not math.isfinite(stamp) or pts.ndim != 2 or pts.shape[1] != 3
                or len(pts) < 3 or not np.isfinite(pts).all() or not np.isfinite(cov).all()
                or not np.allclose(cov, cov.T) or np.linalg.eigvalsh(cov).min() < -1e-9):
            raise ValueError('Invalid local observation')
        stamps = self.stamps.get(self.active, [])
        index = bisect_left(stamps, stamp)
        if index < len(stamps) and abs(stamps[index] - stamp) < STAMP_SNAP_TOLERANCE:
            left = right = stamps[index]
        elif index == 0 or index == len(stamps):
            raise LookupError('Capture time is not bracketed by graph nodes')
        else:
            left, right = stamps[index - 1], stamps[index]
            if right - left > self.max_gap:
                raise LookupError('Graph interpolation gap is too large')
        weight = 0.0 if left == right else (stamp - left) / (right - left)
        anchor = {
            'graph_id': self.graph_id, 'trajectory_id': self.active,
            'left': left, 'right': right, 'weight': weight, 'stamp': stamp,
            'points': FrozenPoints(pts),
            'covariance': cov.ravel().tolist(),
        }
        projected, cov, pose = self.project(anchor)
        anchor['pose'] = pose
        return anchor, projected, cov

    def pose(self, anchor):
        """Resolve the original two nodes without moving their time association."""
        if anchor['graph_id'] != self.graph_id:
            raise ValueError('Pose graph session mismatch')
        nodes = self.nodes.get(anchor['trajectory_id'], {})
        if anchor.get('frozen'):
            # Final pose of trimmed nodes; void once the trajectory is deleted or
            # restarted after this evidence was captured.
            if not nodes or anchor['left'] < self.epoch.get(anchor['trajectory_id'], -math.inf):
                raise LookupError('Frozen anchor trajectory was deleted/restarted')
            pose = np.asarray(anchor['pose'], dtype=float).reshape(3, 4)
            return pose[:, :3], pose[:, 3]
        if anchor['left'] not in nodes or anchor['right'] not in nodes:
            raise LookupError('Anchored graph nodes were removed/unavailable')
        rotation, translation = interpolate_pose(
            nodes[anchor['left']], nodes[anchor['right']], anchor['weight'])
        return rotation, translation

    def project(self, anchor):
        """Recompute from local evidence, so repeated corrections cannot drift."""
        rotation, translation = self.pose(anchor)
        points = np.asarray(anchor['points']) @ rotation.T + translation
        cov = rotation @ np.asarray(anchor['covariance']).reshape(3, 3) @ rotation.T
        pose = np.column_stack((rotation, translation)).ravel().tolist()
        return points, cov, pose
