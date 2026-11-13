#!/usr/bin/env python3
"""Reproducible offline gap-proposal figures from recorded dual LiDAR scans.

No ROS node, publication, or production planner modification. Source the ROS
and workspace setup files before running. All computed paths are counterfactual.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import replace
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'tools'))
from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mecanum_model import rollout
from dra_mppi.mppi_controller import MppiController
from dra_mppi.guide_conditioned import GuideConditionedOptimizer

RES, EXTENT = .04, 4.0
RADIUS, MARGIN = .20, .05
LEVELS = (.5, 1., 1.5)
TOPICS = ['/scan_front_carto', '/scan_rear_carto', '/odometry/filtered',
          '/global_path', '/dra_mppi/reference_trajectory', '/predictions',
          '/dra_mppi/predictions_used', '/traversable_map_grid', '/tf', '/tf_static']


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def yaw(q):
    return np.arctan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def matrix(x, y, a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, x], [s, c, y], [0., 0., 1.]])


def transform(points, mat):
    return np.asarray(points) @ mat[:2, :2].T + mat[:2, 2]


def frame_transform(tf, target, source):
    edges = {}
    for (parent, child), v in tf.items():
        p, q = v.transform.translation, v.transform.rotation
        m = matrix(p.x, p.y, yaw(q))
        edges.setdefault(child, []).append((parent, m))
        edges.setdefault(parent, []).append((child, np.linalg.inv(m)))
    queue, seen = [(source, np.eye(3))], set()
    while queue:
        f, mat = queue.pop(0)
        if f == target:
            return mat
        if f in seen:
            continue
        seen.add(f)
        queue.extend((n, m @ mat) for n, m in edges.get(f, []) if n not in seen)
    raise ValueError(f'No recorded TF: {source} -> {target}')


def read_bag(bag):
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()
             if x.name in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    start = reader.get_metadata().starting_time.nanoseconds
    latest, tf, odoms, snapshots = {}, {}, [], []
    next_time = start + 5_000_000_000
    while reader.has_next():
        topic, data, t = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic in ('/tf', '/tf_static'):
            for v in msg.transforms:
                tf[(v.header.frame_id, v.child_frame_id)] = v
            continue
        latest[topic] = (t, msg)
        if topic == '/odometry/filtered':
            odoms.append((t, msg))
        if topic == '/scan_front_carto' and t >= next_time:
            if all(n in latest for n in TOPICS[:5]):
                snapshots.append(dict(record_ns=t, elapsed=(t-start)*1e-9,
                                      topics=latest.copy(), tf=tf.copy()))
            next_time = t + 2_000_000_000
    return dict(bag=str(bag), start_ns=start, snapshots=snapshots, odoms=odoms)


def prepare_snapshot(data, snap):
    """Deskew each finite beam using only odometry received by snapshot time."""
    od = [m for t, m in data['odoms'] if t <= snap['record_ns']]
    ts = np.array([stamp(m) for m in od])
    poses = np.array([[m.pose.pose.position.x, m.pose.pose.position.y,
                       yaw(m.pose.pose.orientation)] for m in od])
    order = np.argsort(ts)
    ts, poses = ts[order], poses[order]
    poses[:, 2] = np.unwrap(poses[:, 2])
    epoch = ts[-1]
    pose = poses[-1]
    local_from_odom = np.linalg.inv(matrix(*pose))
    odom = od[-1]
    current = np.array([odom.twist.twist.linear.x, odom.twist.twist.linear.y,
                        odom.twist.twist.angular.z])
    n = round(2*EXTENT/RES)
    free, occupied = np.zeros((n, n), np.uint8), np.zeros((n, n), np.uint8)
    scans = []

    def pixel(xy):
        return np.floor((xy+EXTENT)/RES).astype(np.int32)

    for topic in TOPICS[:2]:
        _, msg = snap['topics'][topic]
        ranges = np.asarray(msg.ranges, float)
        angles = msg.angle_min + np.arange(len(ranges))*msg.angle_increment
        beam_times = stamp(msg)+np.arange(len(ranges))*msg.time_increment
        valid = (np.isfinite(ranges) & (ranges >= msg.range_min)
                 & (ranges <= msg.range_max) & (beam_times >= ts[0])
                 & (beam_times <= epoch))
        sensor = frame_transform(snap['tf'], 'base_link', msg.header.frame_id)
        interp = np.column_stack([np.interp(beam_times, ts, poses[:, j])
                                  for j in range(3)])
        cs, sn = np.cos(interp[:, 2]), np.sin(interp[:, 2])
        sensor_offset = sensor[:2, 2]
        origins_odom = interp[:, :2]+np.column_stack((
            cs*sensor_offset[0]-sn*sensor_offset[1],
            sn*sensor_offset[0]+cs*sensor_offset[1]))
        origins = transform(origins_odom, local_from_odom)
        bearing = angles+np.arctan2(sensor[1, 0], sensor[0, 0])+interp[:, 2]-pose[2]
        unit = np.column_stack((np.cos(bearing), np.sin(bearing)))
        # Capped rays still originate in finite measured returns; no inf filling.
        ends = origins + np.minimum(np.where(valid, ranges, 0.), 6.)[:, None]*unit
        hits = origins+np.where(valid, ranges, 0.)[:, None]*unit
        for i in np.flatnonzero(valid[:-1] & valid[1:]):
            # Do not bridge an occlusion edge or an invalid angular sector.
            if abs(ranges[i+1]-ranges[i]) > .25:
                continue
            # Trim the fan to the shorter of adjacent measured depths.
            depth = max(0., min(ranges[i], ranges[i+1], 6.)-.04)
            vertices = np.vstack((origins[i], origins[i+1],
                                  origins[i+1]+depth*unit[i+1],
                                  origins[i]+depth*unit[i]))
            cv2.fillConvexPoly(free, pixel(vertices), 1)
        hit_cells = pixel(hits[valid])
        inside = np.all((hit_cells >= 0) & (hit_cells < n), axis=1)
        hit_cells = hit_cells[inside]
        occupied[hit_cells[:, 1], hit_cells[:, 0]] = 1
        scans.append(dict(topic=topic, frame=msg.header.frame_id, ranges=ranges,
                          angles=angles, beam_times=beam_times, valid=valid,
                          origins=origins, hits=hits, stamp=stamp(msg),
                          angle_increment=msg.angle_increment,
                          time_increment=msg.time_increment))
    # Known robot interior is not an unobserved external obstacle. Hits override.
    coords = -EXTENT+(np.arange(n)+.5)*RES
    xx, yy = np.meshgrid(coords, coords)
    free[xx*xx+yy*yy < RADIUS**2] = 1
    grid = np.full((n, n), -1, np.int16)
    grid[free != 0] = 0
    grid[occupied != 0] = 100
    scan_grid = grid.copy()
    # Dual scanners have offset origins and masked sectors: small seam regions
    # really are unobserved. Supplement only with the recorded, recent map;
    # never silently turn missing scan beams into long free rays.
    map_msg = snap['topics']['/traversable_map_grid'][1]
    if not 0 <= epoch-stamp(map_msg) <= 1.:
        raise ValueError('Recorded traversability grid is stale')
    local_from_map = local_from_odom @ frame_transform(snap['tf'], 'odom', map_msg.header.frame_id)
    origin = map_msg.info.origin
    map_from_grid = matrix(origin.position.x, origin.position.y, yaw(origin.orientation))
    grid_from_local = np.linalg.inv(local_from_map @ map_from_grid)
    locations = transform(np.stack((xx, yy), axis=-1), grid_from_local)
    cell = np.floor(locations/map_msg.info.resolution).astype(int)
    valid_map = ((cell[..., 0] >= 0) & (cell[..., 0] < map_msg.info.width)
                 & (cell[..., 1] >= 0) & (cell[..., 1] < map_msg.info.height))
    recorded = np.asarray(map_msg.data).reshape(map_msg.info.height, map_msg.info.width)
    mapped = np.full_like(grid, -1)
    mapped[valid_map] = recorded[cell[..., 1][valid_map], cell[..., 0][valid_map]]
    map_fill = (grid < 0) & (mapped >= 0) & (mapped < 65)
    grid[map_fill] = 0
    grid[mapped >= 65] = 100
    meta = GridMetadata(n, n, RES, -EXTENT, -EXTENT, 0.)
    field = DistanceField.from_occupancy(grid, meta, frame_id='snapshot_base')
    # Continuous centre clearance approximation used for gap proposals.
    center_clearance = field.clearance - np.sqrt(2)*RES
    angles = np.deg2rad(np.arange(-180., 180., 1.))
    distances = np.arange(0., 2.02, .02)
    rays = np.stack((np.cos(angles), np.sin(angles)), axis=1)[:, None, :]*distances[None, :, None]
    clear = field.query(rays)-np.sqrt(2)*RES-.01 > RADIUS+MARGIN
    prefix = np.logical_and.accumulate(clear, axis=1)
    d_free = np.max(np.where(prefix, distances[None, :], 0.), axis=1)
    msg = snap['topics']['/global_path'][1]
    local_from_path = local_from_odom @ frame_transform(snap['tf'], 'odom', msg.header.frame_id)
    full_path = transform([[v.pose.position.x, v.pose.position.y] for v in msg.poses], local_from_path)
    nearest = int(np.argmin(np.linalg.norm(full_path, axis=1)))
    path = full_path[nearest:nearest+80]
    if len(path) < 2:
        raise ValueError('No remaining recorded global path')
    ref = snap['topics']['/dra_mppi/reference_trajectory'][1]
    ref_times = stamp(ref)+(np.arange(len(ref.references))+1)*ref.dt
    ref_controls = np.array([[v.vx, v.vy, v.yaw_rate] for v in ref.references])
    target_times = epoch+(np.arange(10)+1)*.2
    nominal = np.column_stack([np.interp(target_times, ref_times, ref_controls[:, j])
                              for j in range(3)])
    return dict(snap=snap, epoch=epoch, pose=pose, local_from_odom=local_from_odom,
                current=current, scans=scans, grid=grid, scan_grid=scan_grid,
                map_fill=map_fill, map_age=epoch-stamp(map_msg), field=field,
                center_clearance=center_clearance, angles=angles, d_free=d_free,
                path=path, full_path=full_path, nominal=nominal,
                ref_age=epoch-stamp(ref), ref_times=ref_times, ref_controls=ref_controls)


def circular_runs(mask):
    if np.all(mask):
        return [np.arange(len(mask))]
    if not np.any(mask):
        return []
    starts = np.flatnonzero(mask & ~np.roll(mask, 1))
    groups = []
    for start in starts:
        ids = []
        for k in range(len(mask)):
            i = (start+k) % len(mask)
            if not mask[i]:
                break
            ids.append(i)
        groups.append(np.array(ids))
    return groups


def seed_control(controller, target, current):
    cfg = controller.config
    state, previous, out = np.zeros(3), current.copy(), []
    for _ in range(cfg.horizon_steps):
        delta = target-state[:2]
        distance = np.linalg.norm(delta)
        speed = min(cfg.desired_speed, distance/.6)
        world_v = delta / max(distance, 1e-9)*speed
        c, s = np.cos(state[2]), np.sin(state[2])
        # Gap controls translation; recorded path supplies yaw independently.
        tangent = controller._nearest_path_geometry(state[None, None, :], controller.gap_path)[3][0, 0]
        error = (tangent-state[2]+np.pi) % (2*np.pi)-np.pi
        raw = np.array([c*world_v[0]+s*world_v[1], -s*world_v[0]+c*world_v[1], error/.2])
        u = controller._constrain_controls(raw[None, None], previous)[0, 0]
        state = rollout(state, u[None], cfg.dt)[0]
        out.append(u)
        previous = u
    return np.array(out)


def propose(p):
    cfg = config_from_yaml()
    controller = MppiController(replace(cfg, strategy='standard'))
    controller.gap_path = p['path']
    nominal = controller._constrain_controls(p['nominal'][None], p['current'])[0]
    nominal_path = rollout(np.zeros(3), nominal, cfg.dt)
    nominal_collision = bool(p['field'].trajectory_collisions(
        nominal_path[None, :, :2], np.zeros(2), RADIUS)[0])
    raw = []
    # Visit farther levels first so radial copies of a direction get merged.
    for level in reversed(LEVELS):
        for ids in circular_runs(p['d_free'] >= level):
            if len(ids) < 6:
                continue
            angle = p['angles'][ids[len(ids)//2]]
            raw.append(dict(level=level, ids=ids, angle=angle,
                            width_deg=len(ids), target=level*np.array([np.cos(angle), np.sin(angle)])))
    gaps = []
    for item in raw:
        # Same central direction and overlapping angular support, deeper first.
        if any(abs(np.arctan2(np.sin(item['angle']-g['angle']),
                             np.cos(item['angle']-g['angle']))) < np.deg2rad(12)
               and len(np.intersect1d(item['ids'], g['ids'])) for g in gaps):
            item['merged'] = True
            continue
        g = item.copy()
        g['id'] = f'G{len(gaps)+1}'
        g['controls'] = seed_control(controller, g['target'], p['current'])
        g['trajectory'] = rollout(np.zeros(3), g['controls'], cfg.dt)
        delta = np.linalg.norm(g['trajectory'][:, :2]-nominal_path[:, :2], axis=1)
        g['rms_distance'] = float(np.sqrt(np.mean(delta**2)))
        g['tail_distance'] = float(np.mean(delta[-3:]))
        g['duplicate'] = (not nominal_collision and g['rms_distance'] < .22 and g['tail_distance'] < .30)
        g['collision'] = bool(p['field'].trajectory_collisions(
            g['trajectory'][None, :, :2], np.zeros(2), RADIUS)[0])
        g['reach_error'] = float(np.linalg.norm(g['trajectory'][-1, :2]-g['target']))
        swept = p['field'].trajectory_clearances(g['trajectory'][None, :, :2], np.zeros(2))[0]
        g['clearance'] = float(np.min(swept)-RADIUS)
        # Scores have explicit [0,1] scaling; progress includes backward motion.
        path = p['path']
        cross_track = float(np.min(np.linalg.norm(path-g['trajectory'][-1, :2], axis=1)))
        tangent = path[min(3, len(path)-1)]-path[0]
        tangent /= max(np.linalg.norm(tangent), 1e-9)
        progress = float(np.dot(g['trajectory'][-1, :2], tangent))
        g['progress_m'] = progress
        g['cross_track_m'] = cross_track
        g['features'] = [float(np.clip(progress/1.5, 0, 1)),
                         float(np.clip(g['clearance']/.4, 0, 1)),
                         float(np.clip(g['rms_distance']/.8, 0, 1)),
                         float(np.exp(-cross_track/.6))]
        g['score'] = float(np.dot(g['features'], [.4, .25, .2, .15]))
        if g['duplicate']:
            g['reason'] = 'nominal 중복'
        elif g['collision']:
            g['reason'] = 'seed 충돌'
        elif g['reach_error'] > .35:
            g['reason'] = '도달 부족'
        elif g['score'] < .25:
            g['reason'] = '낮은 가치'
        else:
            g['reason'] = '탐색 가능'
        gaps.append(g)
    selected = []
    for g in sorted(gaps, key=lambda g: -g['score']):
        if g['reason'] != '탐색 가능':
            continue
        if any(np.sqrt(np.mean(np.sum((g['trajectory'][:, :2]-s['trajectory'][:, :2])**2, axis=1))) < .25
               for s in selected):
            g['reason'] = 'gap 간 중복'
        elif len(selected) < 3:
            selected.append(g)
            g['reason'] = '채택'
        else:
            g['reason'] = 'Top-3 밖'
    return dict(controller=controller, config=cfg, nominal=nominal,
                nominal_path=nominal_path, nominal_collision=nominal_collision,
                raw=raw, gaps=gaps, selected=selected)


def recorded_predictions(p, controller):
    """Use the actual node's freshness/filter/interpolation logic without a node."""
    from types import SimpleNamespace
    from rclpy.time import Time
    from rclpy.clock import ClockType
    from dra_mppi.dra_mppi_node import DraMppiNode
    now = Time(nanoseconds=round(p['epoch']*1e9), clock_type=ClockType.ROS_TIME)
    def lookup(target, source):
        m = p['local_from_odom'] @ frame_transform(p['snap']['tf'], 'odom', source)
        return float(m[0, 2]), float(m[1, 2]), float(np.arctan2(m[1, 0], m[0, 0]))
    adapter = SimpleNamespace(
        _prediction_message=copy.deepcopy(p['snap']['topics']['/dra_mppi/predictions_used'][1]),
        prediction_fail_safe=True, _last_prediction_monotonic=time.monotonic(),
        prediction_timeout=.5, planning_frame='snapshot_base', controller=controller,
        prediction_class_names=('person',), prediction_max_semantic_age=1.,
        prediction_max_measurement_age=.5, prediction_require_confirmed=True,
        prediction_allow_recent_lost=True, prediction_lost_grace_time=.30,
        get_clock=lambda: SimpleNamespace(now=lambda: now), _lookup_2d=lookup)
    return DraMppiNode._aligned_predictions(adapter)


class SnapshotGapOptimizer(GuideConditionedOptimizer):
    """Offline adapter: keep existing solver, replace only anchors and allocation."""
    def __init__(self, controller, seeds, names, counts):
        super().__init__(controller)
        self.snapshot_seeds = np.array(seeds)
        self.snapshot_counts = np.array(counts)
        self.names = tuple(names)+('stop',)

    def anchors(self, state, path, current, goal=None):
        return self.snapshot_seeds.copy()

    def allocate(self, active):
        return self.snapshot_counts.copy()


def optimize_snapshot(p, q):
    selected = q['selected']
    seeds = np.array([q['nominal']]+[g['controls'] for g in selected])
    names = ['nominal']+[g['id'] for g in selected]
    counts = [500-100*len(selected)]+[100]*len(selected)
    # Long timeout only for complete offline illustration. This is not a
    # real-time controller trial or evidence that a 200 ms deadline is met.
    cfg = replace(q['config'], strategy='guide_conditioned',
                  guide_angles_deg=tuple(np.arange(len(seeds), dtype=float)),
                  guide_timeout=10.)
    c = MppiController(cfg)
    c._guide_optimizer.shutdown()
    c._guide_optimizer = SnapshotGapOptimizer(c, seeds, names, counts)
    predictions = recorded_predictions(p, c)
    t = time.perf_counter()
    result = c.compute(np.zeros(3), p['path'], p['current'], predictions,
                       visualization_rollouts=500, distance_field=p['field'])
    elapsed = (time.perf_counter()-t)*1000
    c.close()
    assert len(result.sampled_controls) == 500
    # Validate actual sampled and representative controls, not just seed bounds.
    for controls in (result.sampled_controls, result.mode_control_sequences):
        assert np.all(controls <= np.asarray(cfg.control_max)+1e-8)
        assert np.all(controls >= np.asarray(cfg.control_min)-1e-8)
        assert np.max(np.abs(controls[..., 0])+np.abs(controls[..., 1])+
                      cfg.yaw_lever*np.abs(controls[..., 2])) <= cfg.speed_limit_l1+1e-8
        initial = np.broadcast_to(p['current'], (len(controls), 1, 3))
        delta = np.diff(np.concatenate((initial, controls), axis=1), axis=1)
        assert np.all(np.abs(delta) <= np.asarray(cfg.acceleration_limit)*cfg.dt+1e-8)
    if result.safety_status == 'ok':
        assert result.max_collision_probability <= cfg.dynamic_risk.hard_threshold
        assert not p['field'].trajectory_collisions(result.trajectory[None, :, :2], np.zeros(2), RADIUS)[0]
    return dict(result=result, predictions=predictions, names=names,
                counts=counts, seeds=seeds, optimizer_ms=elapsed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, default=Path('/home/smoc/2026amr_ws/bags/mpc_chain_20260923_135325'))
    parser.add_argument('--output', type=Path, default=ROOT/'docs/lidar_gap_flow_2026-11-04')
    parser.add_argument('--offset', type=float)
    parser.add_argument('--inspect', action='store_true')
    args = parser.parse_args()
    data = read_bag(args.bag)
    print(f'Read {len(data["snapshots"])} candidate snapshots', flush=True)
    if args.inspect:
        for snap in data['snapshots']:
            try:
                p = prepare_snapshot(data, snap)
                q = propose(p)
                print(round(snap['elapsed'], 2), 'free',round(float(np.mean(p['grid']==0)),2),
                      'max_d',round(float(p['d_free'].max()),2),'nominal_blocked',q['nominal_collision'],
                      'gaps',[(g['id'],round(np.rad2deg(g['angle'])),g['level'],g['reason'],round(g['score'],2)) for g in q['gaps']], flush=True)
            except Exception as exc:
                print(round(snap['elapsed'],2),type(exc).__name__,str(exc),flush=True)
        return
    snap = min(data['snapshots'], key=lambda s: abs(s['elapsed']-(args.offset or 11.36)))
    t = time.perf_counter()
    p = prepare_snapshot(data, snap)
    prepare_ms = (time.perf_counter()-t)*1000
    t = time.perf_counter()
    q = propose(p)
    proposal_ms = (time.perf_counter()-t)*1000
    opt = optimize_snapshot(p, q)
    opt.update(prepare_ms=prepare_ms, proposal_ms=proposal_ms)
    args.output.mkdir(parents=True, exist_ok=True)
    from lidar_gap_flow_figures import produce
    produce(data, p, q, opt, args.output)


if __name__ == '__main__':
    main()
