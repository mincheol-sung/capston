"""Persistent, bounded object memory with versioned geometric evidence."""

from dataclasses import dataclass
from dataclasses import field
from dataclasses import fields
import json
import math
import os
from pathlib import Path
import uuid
import zipfile

import numpy as np

from semantic_slam.pose_graph import FrozenPoints
from semantic_slam.pose_graph import STAMP_SNAP_TOLERANCE

try:
    from semantic_slam._voxel import first_indices
except ImportError:
    first_indices = None  # Source-only environments retain the exact NumPy path.


# Schema 4 adds ObjectRecord.first_seen. Readers reject a newer schema or a record
# field unknown to its schema: dropping a field and saving again loses it for the
# newer version (a later first_seen would merge two real objects).
SCHEMA = 4


@dataclass
class Observation:
    """A capture-time observation already transformed into the map frame."""

    class_id: int
    class_name: str
    stamp: float
    points: list
    covariance: list
    confidence: float = 1.0
    valid_ratio: float = 1.0
    method: int = 1
    anchor: dict | None = None
    # Capture-time median in the continuous odometry frame, when available.
    odom_position: list | None = None


@dataclass
class ObjectRecord:
    """Stable ID and accumulated visible-surface geometry, not a grasp model."""

    object_id: str
    class_id: int
    class_name: str
    position: list
    center: list
    extent: list
    points: list
    covariance: list
    last_seen: float
    count: int = 1
    alias: str = ''
    method: int = 1
    status: int = 0
    movement_votes: int = 0
    history: list = field(default_factory=list)
    anchors: list = field(default_factory=list)
    support_valid: bool = True
    first_seen: float = 0.0  # 0: unknown, read from a schema 1-3 DB


class ObjectMap:
    """Conservative label/geometry association with one-to-one batch updates."""

    def __init__(
        self,
        map_id,
        path='',
        gate=0.8,
        ambiguity_margin=0.2,
        confirm_count=3,
        min_interval=0.3,
        stale_after=60.0,
        max_objects=200,
        max_points=512,
        voxel=0.05,
        graph_id='',
        max_anchors=16,
        odom_window=5.0,
        merge_distance=1.5,
    ):
        if not map_id.strip():
            raise ValueError('map_id must identify the fixed map and revision')
        if (gate <= 0 or voxel <= 0 or confirm_count < 2 or min_interval <= 0
                or max_objects < 1 or max_points < 3 or ambiguity_margin < 0
                or max_anchors < confirm_count or not merge_distance >= 0):
            raise ValueError('Invalid association/capacity parameters')
        self.map_id, self.path = map_id, Path(path).expanduser() if path else None
        self.gate, self.ambiguity_margin = gate, ambiguity_margin
        self.confirm_count, self.min_interval = confirm_count, min_interval
        self.stale_after = stale_after
        self.max_objects, self.max_points, self.voxel = max_objects, max_points, voxel
        self.objects = {}
        self.rebuild_ids = set()
        self.graph_id, self.max_anchors, self.revision = graph_id, max_anchors, 0
        self.dirty = False
        self.ambiguous_count = 0
        # Last odometry-frame position per object. Short-term association and
        # movement use it instead of the map, whose map->odom correction jumps.
        # Never persisted: the odometry origin does not survive a restart.
        self.odom_window, self.odom_positions = odom_window, {}
        self.merge_distance = merge_distance
        if self.path and self.path.exists():
            self.load()

    def _bounded_points(self, points):
        pts = np.asarray(points, dtype=float).reshape(-1, 3)
        cells = np.ascontiguousarray(np.floor(pts / self.voxel))
        if not len(cells):
            return pts
        lower, upper = cells.min(axis=0), cells.max(axis=0)
        if (np.all(lower >= -(2**52)) and np.all(upper <= 2**52)
                and np.all(upper - lower < 2**21)):
            # Exact collision-free voxel keys for bounded geometry. Rebase before
            # packing, so negative coordinates and large world offsets are valid.
            relative = (cells - lower).astype(np.uint64)
            keys = (relative[:, 0] << 42) | (relative[:, 1] << 21) | relative[:, 2]
            if first_indices is not None:
                indices = np.frombuffer(first_indices(keys), dtype=np.int64)
                pts = pts[indices]
                if len(pts) > self.max_points:
                    pts = pts[np.linspace(0, len(pts) - 1, self.max_points, dtype=int)]
                return pts
            # Quicksort followed by each equal-key group's minimum source index
            # retains exactly the first point, without a stable full-key sort.
            order = np.argsort(keys, kind='quicksort')
            ordered = keys[order]
            starts = np.r_[0, np.flatnonzero(ordered[1:] != ordered[:-1]) + 1]
            indices = np.minimum.reduceat(order, starts)
        else:
            # Preserve numeric row equality for unusually wide/large geometry.
            cells[cells == 0] = 0.0
            keys = cells.view(np.dtype((np.void, cells.dtype.itemsize * 3))).ravel()
            _, indices = np.unique(keys, return_index=True)
        pts = pts[np.sort(indices)]
        if len(pts) > self.max_points:
            pts = pts[np.linspace(0, len(pts) - 1, self.max_points, dtype=int)]
        return pts

    @staticmethod
    def _geometry(points):
        lo, hi = np.quantile(points, [0.05, 0.95], axis=0)
        return (
            np.median(points, axis=0).tolist(),
            ((lo + hi) / 2).tolist(),
            np.maximum(hi - lo, 0.05).tolist(),
        )

    def update(self, observations):
        """Ignore uncertain assignments rather than merge similar nearby chairs."""
        prior_stamps = {key: record.last_seen for key, record in self.objects.items()}
        candidates = []
        for obs in observations:
            if self.graph_id and (
                not obs.anchor or obs.anchor.get('graph_id') != self.graph_id
            ):
                continue
            pts = np.asarray(obs.points, dtype=float)
            cov = np.asarray(obs.covariance, dtype=float)
            quality = np.asarray([obs.confidence, obs.valid_ratio], dtype=float)
            if (
                pts.ndim != 2
                or pts.shape[1] != 3
                or len(pts) < 3
                or not np.isfinite(pts).all()
                or cov.size != 9
                or not np.isfinite(cov).all()
                or not math.isfinite(obs.stamp)
                or obs.stamp <= 0
                or not np.isfinite(quality).all()
                or np.any(quality > 1)
                or obs.confidence < 0.4
                or obs.valid_ratio < 0.2
            ):
                continue
            matrix = cov.reshape(3, 3)
            if not np.allclose(matrix, matrix.T) or np.linalg.eigvalsh(matrix).min() < -1e-9:
                continue
            position = np.median(pts, axis=0)
            _, observed_center, observed_extent = self._geometry(pts)
            matches = []
            for record in self.objects.values():
                if record.class_id != obs.class_id or obs.stamp <= record.last_seen:
                    continue
                motion = self._odom_motion(record.object_id, obs)
                if motion is None:
                    offset = position - record.position
                else:
                    # Compare in odometry; the map may have been re-aligned since.
                    offset = motion
                distance = float(np.linalg.norm(offset))
                box_gap = np.maximum(
                    np.abs(np.array(observed_center) - position
                           - (np.asarray(record.center) - record.position) + offset)
                    - (np.array(observed_extent) + record.extent) / 2, 0)
                shape_distance = float(np.linalg.norm(box_gap))
                # The association gate is bounded even for large observed boxes.
                if distance <= self.gate and shape_distance <= self.gate * 0.5:
                    matches.append((distance + 0.35 * shape_distance, record.object_id))
            matches.sort()
            if len(matches) > 1 and matches[1][0] - matches[0][0] < self.ambiguity_margin:
                self.ambiguous_count += 1
                continue
            candidates.append(
                (
                    matches[0][0] if matches else float('inf'),
                    matches[0][1] if matches else None,
                    obs,
                )
            )
        used = set()
        updated = []
        for distance, object_id, obs in sorted(candidates, key=lambda x: x[0]):
            if object_id in used:
                self.ambiguous_count += 1
                continue
            if object_id is None:
                # Suppress duplicate/reordered observations of an existing object.
                pos = np.median(obs.points, axis=0)
                if any(
                    r.class_id == obs.class_id
                    and np.linalg.norm(pos - r.position)
                    <= (
                        self.gate
                        if r.object_id in prior_stamps and obs.stamp <= prior_stamps[r.object_id]
                        else 0.15
                    )
                    for r in self.objects.values()
                ):
                    continue
                if len(self.objects) >= self.max_objects:
                    continue
                pts = self._bounded_points(obs.points)
                position, center, extent = self._geometry(pts)
                object_id = str(uuid.uuid4())
                self.objects[object_id] = ObjectRecord(
                    object_id,
                    obs.class_id,
                    obs.class_name,
                    position,
                    center,
                    extent,
                    pts.tolist(),
                    np.asarray(obs.covariance, dtype=float).ravel().tolist(),
                    obs.stamp,
                    method=obs.method,
                    history=[[obs.stamp, *position]],
                    anchors=[obs.anchor] if obs.anchor else [],
                    first_seen=obs.stamp,
                )
            else:
                record = self.objects[object_id]
                dt = obs.stamp - record.last_seen
                if dt < self.min_interval:
                    continue
                current = np.median(obs.points, axis=0)
                motion = self._odom_motion(object_id, obs)
                displacement = float(np.linalg.norm(
                    current - record.position if motion is None else motion))
                speed = displacement / dt
                moving = displacement > 0.20 and speed > 0.25
                # The object stayed in odometry but not in the map: map->odom jumped.
                # Re-anchor at the new map pose; old evidence is in the old alignment.
                realigned = (motion is not None and not moving
                             and np.linalg.norm(current - record.position - motion) > 0.20)
                record.movement_votes = (
                    min(3, record.movement_votes + 1)
                    if moving
                    else max(0, record.movement_votes - 1)
                )
                fresh = moving or realigned
                pts = self._bounded_points(obs.points if fresh else record.points + obs.points)
                _, record.center, record.extent = self._geometry(pts)
                record.points = pts.tolist()
                record.history = ([] if realigned else record.history)
                record.history = (record.history + [[obs.stamp, *current.tolist()]])[-8:]
                record.position = np.median(np.array(record.history)[:, 1:], axis=0).tolist()
                if moving:
                    record.position = current.tolist()
                record.covariance = np.asarray(obs.covariance, dtype=float).ravel().tolist()
                record.last_seen = obs.stamp
                record.count += 1
                record.method = obs.method
                record.status = (
                    3
                    if record.movement_votes >= 2
                    else (1 if record.count >= self.confirm_count else 0)
                )
                if obs.anchor:
                    # Never blend a moving object's old surface into its new pose.
                    record.anchors = ([obs.anchor] if moving else
                                      (record.anchors + [obs.anchor])[-self.max_anchors:])
            if obs.odom_position is not None:
                self.odom_positions[object_id] = (obs.stamp, np.asarray(obs.odom_position))
            used.add(object_id)
            updated.append(object_id)
            if obs.anchor:
                self.rebuild_ids.add(object_id)
            self.dirty = True
        merged = self._merge_duplicates(updated)
        return [merged.get(object_id, object_id) for object_id in updated]

    @staticmethod
    def _first_seen(record):
        # Not estimated from history: it keeps the last 8 entries, so the estimate
        # is later than the truth and would pass the lifetime-overlap test.
        return record.first_seen if record.first_seen > 0 else None

    def _merge_duplicates(self, object_ids):
        """
        Fold a newly confirmed record into the older one it re-created.

        A map->odom jump while an object is out of view, past the odometry
        window, re-creates it beyond the association gate; confirmed objects are
        never deleted, so the copy would stay. Two physical objects are told apart
        by overlapping lifetimes: the older one was seen after the newer one first
        appeared. Every co-observation in one frame is such an overlap.
        Correction mode is excluded: its geometry comes from graph reprojection.
        """
        merged = {}
        if self.merge_distance <= 0 or self.graph_id:
            return merged
        for object_id in object_ids:
            newer = self.objects.get(object_id)
            if newer is None or newer.count < self.confirm_count:
                continue
            first_seen = self._first_seen(newer)
            if first_seen is None:
                continue
            older = [
                record for record in self.objects.values()
                if record is not newer
                and record.class_id == newer.class_id
                and record.count >= self.confirm_count
                and record.last_seen < first_seen
                and np.linalg.norm(np.subtract(record.position, newer.position))
                <= self.merge_distance
                # Distinct operator aliases assert two objects.
                and not (record.alias and newer.alias and record.alias != newer.alias)
            ]
            if len(older) > 1:
                self.ambiguous_count += 1
            if len(older) != 1:
                continue
            target = older[0]
            # Identity and alias from the original; geometry and state from the
            # newer record, which is in the current map alignment.
            for name in ('position', 'center', 'extent', 'points', 'covariance', 'history',
                         'last_seen', 'method', 'status', 'movement_votes'):
                setattr(target, name, getattr(newer, name))
            target.count += newer.count
            target.alias = target.alias or newer.alias
            if newer.object_id in self.odom_positions:
                self.odom_positions[target.object_id] = self.odom_positions.pop(newer.object_id)
            del self.objects[newer.object_id]
            merged[newer.object_id] = target.object_id
            self.dirty = True
        return merged

    def _odom_motion(self, object_id, obs):
        """Object displacement in odometry since it was last seen, or None."""
        seen = self.odom_positions.get(object_id)
        if obs.odom_position is None or seen is None:
            return None
        stamp, odom_position = seen
        if not 0 < obs.stamp - stamp <= self.odom_window:
            return None
        return np.asarray(obs.odom_position, dtype=float) - odom_position

    def reproject(self, graph):
        """Refuse missing support; rebuild retained evidence without reassociation."""
        if graph.graph_id != self.graph_id:
            raise ValueError('Pose graph session mismatch')
        changed = False
        # Objects often share graph nodes. Cache only for this immutable snapshot
        # traversal; frozen anchors retain their own final pose and are not shared.
        poses = {}
        for record in self.objects.values():
            supported, pose_changed = bool(record.anchors), False
            resolved = []
            for anchor in record.anchors:
                # Bind/load normally freeze evidence once. Also accept legacy
                # in-memory anchors without a cache that could become stale.
                anchor['points'] = FrozenPoints(anchor['points'])
                try:
                    key = (anchor['graph_id'], anchor['trajectory_id'], anchor['left'],
                           anchor['right'], anchor['weight'])
                    if anchor.get('frozen'):
                        rotation, translation = graph.pose(anchor)
                        pose = np.column_stack((rotation, translation)).ravel().tolist()
                    else:
                        if key not in poses:
                            rotation, translation = graph.pose(anchor)
                            pose = np.column_stack((rotation, translation)).ravel().tolist()
                            poses[key] = rotation, translation, pose
                        rotation, translation, pose = poses[key]
                except LookupError:
                    if graph.trimmed(anchor):
                        # Trimmed nodes are never re-optimized: keep the last pose.
                        anchor['frozen'] = True
                        self.dirty = True
                        rotation, translation = graph.pose(anchor)
                        pose = np.column_stack((rotation, translation)).ravel().tolist()
                    else:
                        supported = False
                        continue
                resolved.append((anchor, rotation, translation))
                previous_pose = anchor['pose']
                if len(previous_pose) != 12:
                    raise ValueError('Invalid anchor pose shape')
                if not all(abs(a - b) <= 1e-6 for a, b in zip(pose, previous_pose)):
                    pose_changed = True
                    anchor['pose'] = list(pose)
            support_changed = record.support_valid != supported
            changed = changed or pose_changed or support_changed
            if not (pose_changed or support_changed or record.object_id in self.rebuild_ids):
                continue
            record.support_valid = supported
            self.rebuild_ids.discard(record.object_id)
            if supported:
                if len({len(anchor['points']) for anchor, _, _ in resolved}) == 1:
                    # One dense conversion/transform avoids per-anchor arrays and
                    # the subsequent stack/concatenate copies at maximum capacity.
                    raw = np.asarray([anchor['points'] for anchor, _, _ in resolved])
                    if raw.shape != (len(resolved), len(resolved[0][0]['points']), 3):
                        raise ValueError('Invalid anchor point shape')
                    rotations = np.stack([rotation for _, rotation, _ in resolved])
                    translations = np.array([translation for _, _, translation in resolved])
                    projected = raw @ rotations.swapaxes(1, 2) + translations[:, None, :]
                    medians = np.median(projected, axis=1)
                    all_points = projected.reshape(-1, 3)
                else:
                    points = [np.asarray(anchor['points']) @ rotation.T + translation
                              for anchor, rotation, translation in resolved]
                    medians = [np.median(pts, axis=0) for pts in points]
                    all_points = np.concatenate(points)
                history = [[anchor['stamp'], *median.tolist()]
                           for (anchor, _, _), median in zip(resolved, medians)]
                anchor, rotation, _ = resolved[-1]
                latest_cov = rotation @ np.asarray(anchor['covariance']).reshape(3, 3)
                latest_cov = latest_cov @ rotation.T
                record.points = self._bounded_points(all_points).tolist()
                _, record.center, record.extent = self._geometry(record.points)
                record.history = history[-8:]
                record.position = np.median(np.asarray(record.history)[:, 1:], axis=0).tolist()
                record.covariance = latest_cov.ravel().tolist()
                # Geometry correction is not evidence that furniture moved or was seen again.
                if record.status != 3:
                    record.status = 1 if record.count >= self.confirm_count else 0
            elif record.status != 3:
                record.status = 2
        if changed:
            self.revision += 1
            self.dirty = True
        return changed

    def prune_anchors(self, trajectories):
        """
        Drop anchors on deleted trajectories so remaining evidence can support the object.

        An object whose anchors are all lost keeps them: it stays unsupported (STALE),
        and the saved DB still satisfies the raw-anchor invariant on reload.
        """
        if not trajectories:
            return
        for record in self.objects.values():
            kept = [a for a in record.anchors if a['trajectory_id'] not in trajectories]
            if kept and len(kept) < len(record.anchors):
                record.anchors = kept
                self.rebuild_ids.add(record.object_id)
                self.dirty = True

    def age(self, now):
        """Expire tentative noise; retain confirmed furniture when occluded."""
        for object_id, record in list(self.objects.items()):
            if record.count < self.confirm_count and now - record.last_seen > 10.0:
                del self.objects[object_id]
                self.rebuild_ids.discard(object_id)
                self.odom_positions.pop(object_id, None)
                self.dirty = True
            elif record.status != 3 and now - record.last_seen > self.stale_after:
                record.status = 2

    def set_alias(self, object_id, alias):
        alias = alias.strip()
        if object_id not in self.objects:
            raise ValueError('Unknown object ID')
        if len(alias) > 80 or (
            alias
            and any(r.alias == alias and r.object_id != object_id for r in self.objects.values())
        ):
            raise ValueError('Alias must be unique and at most 80 characters')
        self.objects[object_id].alias = alias
        self.dirty = True

    def relations(self, distance=2.0):
        records = [
            r for r in self.objects.values() if r.count >= self.confirm_count and r.status != 3
        ]
        if len(records) < 2:
            return []
        positions = np.asarray([record.position[:2] for record in records])
        first, second = np.triu_indices(len(records), 1)
        distances = np.linalg.norm(positions[first] - positions[second], axis=1)
        return [(records[i].object_id, records[j].object_id, 'near', float(d))
                for i, j, d in zip(first, second, distances) if d <= distance]

    def save(self):
        """
        Commit one uncompressed schema-4 archive under the owning node's lock.

        The single-threaded executor must not mutate this map during the save.
        Metadata is shallow-copied; large point lists are converted only to float64.
        """
        if self.path is None or not self.dirty:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(self.path.suffix + '.tmp')
        opened = False
        try:
            # The owning node holds the DB lock: reclaim this DB's orphan temporary.
            with temporary.open('wb') as stream:
                opened = True
                metadata, blocks = self._archive_data()
                with zipfile.ZipFile(stream, 'w', compression=zipfile.ZIP_STORED) as archive:
                    archive.writestr('metadata.json', json.dumps(
                        metadata, ensure_ascii=False, allow_nan=False,
                        separators=(',', ':')).encode('utf-8'))
                    with archive.open('points.bin', 'w', force_zip64=True) as points:
                        for block in blocks:
                            points.write(memoryview(block).cast('B'))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0))
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if opened:
                temporary.unlink(missing_ok=True)
        self.dirty = False

    @staticmethod
    def _point_array(points):
        if isinstance(points, FrozenPoints):
            # Its immutable bytes were shape/finite-checked at construction.
            return np.asarray(points)
        array = np.asarray(points, dtype='<f8')
        if (array.ndim != 2 or array.shape[1] != 3 or len(array) < 1
                or not np.isfinite(array).all()):
            raise ValueError('Object DB contains invalid point geometry')
        return array

    def _validate_record(self, record):
        if not isinstance(record.object_id, str) or not record.object_id:
            raise ValueError('Object DB contains invalid object ID')
        for name, shape in (('position', (3,)), ('center', (3,)), ('extent', (3,)),
                            ('covariance', (9,))):
            values = np.asarray(getattr(record, name), dtype=float)
            if values.shape != shape or not np.isfinite(values).all():
                raise ValueError(f'Object DB contains invalid {name}')
        history = np.asarray(record.history, dtype=float)
        if history.size and (history.ndim != 2 or history.shape[1] != 4
                             or not np.isfinite(history).all()):
            raise ValueError('Object DB contains invalid history')
        if not math.isfinite(record.last_seen) or not (
                math.isfinite(record.first_seen) and record.first_seen >= 0):
            raise ValueError('Object DB contains invalid observation time')
        if not isinstance(record.anchors, list) or len(record.anchors) > self.max_anchors:
            raise ValueError('Object DB exceeds anchor capacity')
        if self.graph_id and not record.anchors:
            raise ValueError('Legacy object has no raw anchors; use a new DB')
        for name, width in (('pose', 12), ('covariance', 9)):
            values = np.asarray([anchor[name] for anchor in record.anchors], dtype=float)
            if record.anchors and (values.shape != (len(record.anchors), width)
                                   or not np.isfinite(values).all()):
                raise ValueError(f'Object DB contains invalid anchor {name}')
        for anchor in record.anchors:
            if anchor['graph_id'] != self.graph_id:
                raise ValueError('Object DB anchor graph session mismatch')
            left, right, stamp = anchor['left'], anchor['right'], anchor['stamp']
            if not np.isfinite([left, right, stamp, anchor['weight']]).all():
                raise ValueError('Object DB contains invalid anchor time')
            in_interval = 0 < left < right and left <= stamp <= right
            snapped = (0 < left == right and stamp > 0 and anchor['weight'] == 0
                       and abs(stamp - left) < STAMP_SNAP_TOLERANCE)
            if (type(anchor['trajectory_id']) is not int or anchor['trajectory_id'] < 0
                    or not 0 <= anchor['weight'] <= 1
                    or not (in_interval or snapped)):
                raise ValueError('Object DB contains invalid anchor association')

    def _archive_data(self):
        blocks, items, offset = [], [], 0

        def reference(points):
            nonlocal offset
            block = self._point_array(points)
            ref = {'offset': offset, 'count': len(block), 'dtype': '<f8',
                   'shape': [len(block), 3]}
            offset += block.nbytes
            blocks.append(block)
            return ref

        if len(self.objects) > self.max_objects:
            raise ValueError('Object DB exceeds configured capacity')
        if type(self.revision) is not int or self.revision < 0:
            raise ValueError('Object DB contains invalid revision')
        for key, record in self.objects.items():
            self._validate_record(record)
            if key != record.object_id:
                raise ValueError('Object DB object ID/key mismatch')
            item = dict(vars(record))
            item['points'] = reference(record.points)
            item['anchors'] = [dict(anchor, points=reference(anchor['points']))
                               for anchor in record.anchors]
            items.append(item)
        return ({'schema': SCHEMA, 'generation': uuid.uuid4().hex, 'map_id': self.map_id,
                 'graph_id': self.graph_id, 'revision': self.revision,
                 'points_bytes': offset, 'objects': items}, blocks)

    def _read_archive(self, stream):
        with zipfile.ZipFile(stream) as archive:
            if (sorted(archive.namelist()) != ['metadata.json', 'points.bin']
                    or any(info.compress_type != zipfile.ZIP_STORED
                           for info in archive.infolist())):
                raise ValueError('Object DB archive entries/encoding mismatch')
            data = json.loads(archive.read('metadata.json'))
            self._check_newer(data)
            if (type(data.get('schema')) is not int or data['schema'] not in (3, SCHEMA)
                    or not isinstance(data.get('generation'), str)):
                raise ValueError('Object DB archive schema/generation mismatch')
            uuid.UUID(hex=data['generation'])
            info = archive.getinfo('points.bin')
            if (type(data.get('points_bytes')) is not int
                    or data['points_bytes'] != info.file_size):
                raise ValueError('Object DB binary length mismatch')
            if not isinstance(data['objects'], list) or len(data['objects']) > self.max_objects:
                raise ValueError('Object DB exceeds configured capacity')
            # Full entry reads verify ZIP CRC-32; no additional full-payload hash.
            raw = archive.read('points.bin')
        offset = 0

        def points(ref, immutable=False):
            nonlocal offset
            count = ref['count']
            if (type(count) is not int or count < 1 or ref['dtype'] != '<f8'
                    or ref['shape'] != [count, 3] or type(ref['offset']) is not int
                    or ref['offset'] != offset or count * 24 > len(raw) - offset):
                raise ValueError('Object DB point offset/shape/dtype mismatch')
            array = np.frombuffer(raw, dtype='<f8', count=count * 3, offset=offset).reshape(-1, 3)
            offset += count * 24
            validated = self._point_array(array)
            return FrozenPoints(validated) if immutable else validated.tolist()

        for item in data['objects']:
            item['points'] = points(item['points'])
            if len(item['anchors']) > self.max_anchors:
                raise ValueError('Object DB exceeds anchor capacity')
            for anchor in item['anchors']:
                anchor['points'] = points(anchor['points'], immutable=True)
        if offset != len(raw):
            raise ValueError('Object DB contains unreferenced point bytes')
        return data

    @staticmethod
    def _check_newer(data):
        schema = data.get('schema')
        if type(schema) is int and schema > SCHEMA:
            raise ValueError(
                f'Object DB schema {schema} was written by a newer version (this reads '
                f'up to {SCHEMA}); open it with that version or use a separate DB')

    @staticmethod
    def _record_fields(item, schema):
        known = {f.name for f in fields(ObjectRecord)}
        if schema < 4:
            known.discard('first_seen')
        unknown = sorted(set(item) - known)
        if unknown:
            raise ValueError(f'Object DB record has fields unknown to schema {schema}: {unknown}')
        return item

    def load(self):
        """Validate legacy JSON or a schema-3/4 archive before replacing any live state."""
        try:
            with self.path.open('rb') as stream:
                magic = stream.read(4)
                stream.seek(0)
                data = (self._read_archive(stream)
                        if magic.startswith(b'PK') else json.load(stream))
            self._check_newer(data)
            if (type(data.get('schema')) is not int or data['schema'] not in (1, 2, 3, SCHEMA)
                    or data.get('map_id') != self.map_id):
                raise ValueError(
                    'Object DB map/schema mismatch; use a separate DB for this revision')
            if data['schema'] >= 3 and not magic.startswith(b'PK'):
                raise ValueError(f'Object DB schema {data["schema"]} requires an archive')
            if data.get('graph_id', '') != self.graph_id:
                raise ValueError('Object DB graph session mismatch; use a separate DB')
            revision = data.get('revision', 0)
            if type(revision) is not int or revision < 0:
                raise ValueError('Object DB contains invalid revision')
            if not isinstance(data['objects'], list) or len(data['objects']) > self.max_objects:
                raise ValueError('Object DB exceeds configured capacity')
            objects, rebuild_ids = {}, set()
            for item in data['objects']:
                record = ObjectRecord(**self._record_fields(item, data['schema']))
                self._validate_record(record)
                self._point_array(record.points)
                # Legacy behavior bounds geometry; schema 3+ preserves exact stored points
                # unless the operator deliberately reduces the configured point capacity.
                if data['schema'] < 3 or len(record.points) > self.max_points:
                    record.points = self._bounded_points(record.points).tolist()
                for anchor in record.anchors:
                    anchor['points'] = FrozenPoints(self._point_array(anchor['points']))
                if record.object_id in objects:
                    raise ValueError('Object DB contains duplicate object IDs')
                objects[record.object_id] = record
                if record.anchors:
                    rebuild_ids.add(record.object_id)
        except (KeyError, TypeError, AttributeError, OverflowError,
                UnicodeError, zipfile.BadZipFile) as exc:
            raise ValueError(f'Invalid object DB: {exc}') from exc
        self.objects, self.revision, self.rebuild_ids = objects, revision, rebuild_ids
