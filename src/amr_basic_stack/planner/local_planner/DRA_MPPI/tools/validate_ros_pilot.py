#!/usr/bin/env python3
"""Isolated real ROS timer/DDS benchmark with synthetic inputs, never actuators.

Run after sourcing matching amr_interfaces. Default is a short pilot.
For a full single condition use --cycles 10000 --warmup 50. This does not
simulate perception computation or closed-loop navigation.
"""

import argparse
import csv
import hashlib
import resource
from collections import Counter
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def stats(values):
    data = np.asarray([v for v in values if v is not None], dtype=float)
    if not len(data):
        return None
    return dict(count=len(data), mean=float(np.mean(data)),
                **dict(zip(('p50', 'p95', 'p99', 'max'),
                           np.percentile(data, [50, 95, 99, 100], method='linear').tolist())))


def metadata():
    def command(args):
        return subprocess.run(args, capture_output=True, text=True).stdout.strip()
    return {
        'platform': platform.platform(), 'python': platform.python_version(),
        'ros': os.environ.get('ROS_DISTRO'), 'domain': os.environ['ROS_DOMAIN_ID'],
        'cpu': command(['lscpu']), 'memory': Path('/proc/meminfo').read_text(),
        'load': os.getloadavg(), 'processes': command(['ps', '-eo', 'pid,comm,pcpu,pmem']),
        'thermal': {str(p): p.read_text().strip()
                    for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')},
        'commit': command(['git', 'rev-parse', 'HEAD']),
        'executor': 'SingleThreadedExecutor; fixtures/subscriber in separate process',
        'clock': 'ROS timer; monotonic wall measurement; use_sim_time=false',
        'load_scope': 'planner plus synthetic message fixture; no perception inference',
    }


def helper(args, parameters):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Path as PathMsg, Odometry
    from geometry_msgs.msg import PoseStamped
    from amr_interfaces.msg import (
        TraversableMap, MultiModalObjectPredictionArray, MultiModalObjectPrediction,
        MotionModePrediction, GaussianPrediction2D, PlanarReferenceArray,
    )
    rclpy.init(args=[])
    node = Node('guide_validation_fixture')
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    path_pub = node.create_publisher(PathMsg, parameters['global_path_topic'], latched)
    map_pub = node.create_publisher(TraversableMap, parameters['traversability.topic'], latched)
    odom_pub = node.create_publisher(Odometry, parameters['odometry_topic'], 10)
    pred_pub = node.create_publisher(
        MultiModalObjectPredictionArray, parameters['predictions_topic'], 10)
    stream = (args.output/'receive.jsonl').open('w')
    last = None

    def receive(message):
        nonlocal last
        now = time.monotonic()
        stream.write(json.dumps({'received': now, 'interval_ms': None if last is None else
                                  1000*(now-last), 'stamp_ns':
                                  message.header.stamp.sec*10**9+message.header.stamp.nanosec})+'\n')
        last = now

    node.create_subscription(PlanarReferenceArray, parameters['reference_trajectory_topic'], receive, 100)
    path = PathMsg()
    path.header.frame_id = 'odom'
    points = [(0., 0.), (.4, 1.5), (2., 1.5), (3., 0.)] if args.detour else [(0., 0.), (3., 0.)]
    for x, y in points:
        p = PoseStamped()
        p.header.frame_id = 'odom'
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.w = 1.
        path.poses.append(p)
    grid = np.zeros((200, 200), dtype=np.int8)
    grid[:3, :] = 100
    if args.detour:
        grid[80:120, 118:122] = 100
    m = TraversableMap()
    m.header.frame_id = m.grid.header.frame_id = 'odom'
    m.grid.info.width = m.grid.info.height = 200
    m.grid.info.resolution = .05
    m.grid.info.origin.position.x = m.grid.info.origin.position.y = -5.
    m.grid.info.origin.orientation.w = 1.
    m.grid.data = grid.ravel().tolist()

    def publish_fast():
        now = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = now, 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        predictions = MultiModalObjectPredictionArray()
        predictions.header.stamp, predictions.header.frame_id = now, 'odom'
        for i in range(args.obstacles):
            obj = MultiModalObjectPrediction()
            obj.track_id, obj.class_id, obj.class_name = i+1, 0, 'person'
            obj.radius, obj.motion_state, obj.lifecycle_state = .25, 2, 1
            obj.class_confidence = .9
            obj.time_offsets = np.linspace(0., 3., 16).tolist()
            for mode_index in range(2):
                mode = MotionModePrediction()
                mode.model_type = mode_index
                for t in obj.time_offsets:
                    point = GaussianPrediction2D()
                    point.mean_x = .8+(i % 5)*.5+(.12 if mode_index == 0 else -.08)*t
                    point.mean_y = -1.5+(i//5)*.9+.15*t
                    point.covariance = ([.05, .025, .025, .025]
                        if args.covariance == 'anisotropic' or (args.covariance == 'mixed' and i % 2)
                        else [.04, 0., 0., .04])
                    point.weight = .6 if mode_index == 0 else .4
                    mode.points.append(point)
                obj.modes.append(mode)
            predictions.objects.append(obj)
        pred_pub.publish(predictions)

    def publish_map():
        stamp = node.get_clock().now().to_msg()
        path.header.stamp = stamp
        path_pub.publish(path)
        m.header.stamp = m.grid.header.stamp = m.sensor_watermark = stamp
        map_pub.publish(m)

    node.create_timer(.05, publish_fast)
    node.create_timer(.5, publish_map)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cycles', type=int, default=30)
    parser.add_argument('--warmup', type=int, default=10)
    parser.add_argument('--obstacles', type=int, choices=(0, 5, 10, 20, 21), default=0)
    parser.add_argument('--detour', action='store_true')
    parser.add_argument('--static-off', action='store_true')
    parser.add_argument('--strategy', choices=('standard', 'guide_conditioned', 'single_anchor'))
    parser.add_argument('--rollouts', type=int)
    parser.add_argument('--domain', type=int, default=173)
    parser.add_argument('--workers', type=int, choices=(1, 2, 5))
    parser.add_argument('--backend', choices=('batch', 'split'))
    parser.add_argument('--timeout', type=float)
    parser.add_argument('--seed', type=int)
    parser.add_argument('--density', type=float)
    parser.add_argument('--mc-budget-multiplier', type=int, default=1)
    parser.add_argument('--risk-mode', choices=('monte_carlo', 'gaussian_disk'))
    parser.add_argument('--final-risk-mode', choices=('inherit', 'monte_carlo', 'gaussian_disk'))
    parser.add_argument('--covariance', choices=('isotropic', 'anisotropic', 'mixed'), default='isotropic')
    parser.add_argument('--frozen', action='store_true', help='reset nominal each callback for matched candidates')

    parser.add_argument('--helper', action='store_true')
    args = parser.parse_args()
    if args.domain not in range(170, 180) or args.cycles < 1 or args.warmup < 0:
        parser.error('use isolated domain 170..179 and positive cycles')
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
                      OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1')
    args.output.mkdir(parents=True, exist_ok=True)
    parameters = yaml.safe_load((ROOT/'config/dra_mppi.yaml').read_text())['dra_mppi_node']['ros__parameters']
    for key, value in list(parameters.items()):
        if key.endswith('topic') and isinstance(value, str):
            parameters[key] = '/guide_validation'+value
    parameters.update({'publish_cmd_vel': False, 'static_cost.enabled': not args.static_off,
                       'timing.jsonl_path': str(args.output/'cycles.jsonl'), 'use_sim_time': False})
    if args.strategy:
        parameters['mppi.strategy'] = (
            'guide_conditioned' if args.strategy == 'single_anchor' else args.strategy)
        if args.strategy == 'single_anchor':
            parameters['mppi.guide_angles_deg'] = [0.]
    if args.rollouts is not None:
        parameters['mppi.num_rollouts'] = args.rollouts
        parameters['mppi.guide_num_rollouts'] = args.rollouts
    if args.workers is not None:
        parameters['dynamic_risk.guide_parallel_workers'] = args.workers
    if args.backend is not None:
        parameters['mppi.guide_evaluation'] = args.backend
    if args.timeout is not None:
        parameters['mppi.guide_timeout'] = args.timeout
    if args.seed is not None:
        parameters['mppi.seed'] = args.seed
        parameters['dynamic_risk.seed'] = args.seed+10
    if args.risk_mode is not None:
        parameters['dynamic_risk.mode'] = args.risk_mode
    if args.final_risk_mode is not None:
        parameters['dynamic_risk.final_mode'] = args.final_risk_mode
    if args.density is not None:
        for prefix in ('mc', 'guide_mc', 'final_mc'):
            parameters['dynamic_risk.'+prefix+'_sample_density'] = args.density
            parameters['dynamic_risk.'+prefix+'_samples_min'] = 2048
            parameters['dynamic_risk.'+prefix+'_samples_max'] = 131072
    if args.mc_budget_multiplier < 1:
        parser.error('mc-budget-multiplier must be positive')
    if args.mc_budget_multiplier != 1:
        for prefix in ('mc', 'guide_mc', 'final_mc'):
            for suffix in ('sample_density', 'samples_min', 'samples_max'):
                key = 'dynamic_risk.'+prefix+'_'+suffix
                parameters[key] *= args.mc_budget_multiplier
    if args.helper:
        helper(args, parameters)
        return
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from dra_mppi.dra_mppi_node import DraMppiNode
    config_file = args.output/'requested_parameters.yaml'
    config_file.write_text(yaml.safe_dump({'dra_mppi_node': {'ros__parameters': parameters}}))
    rclpy.init(args=['--ros-args', '--params-file', str(config_file)])
    node = DraMppiNode()
    if args.frozen:
        original_compute = node.controller.compute
        def frozen_compute(*a, **kw):
            node.controller.reset()
            return original_compute(*a, **kw)
        node.controller.compute = frozen_compute
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    actual = {name: node.get_parameter(name).value
              for name in node.list_parameters([], depth=10).names}
    (args.output/'actual_parameters.yaml').write_text(
        yaml.safe_dump({'dra_mppi_node': {'ros__parameters': actual}}))
    (args.output/'environment.json').write_text(json.dumps(metadata(), indent=2))
    (args.output/'source_sha256.json').write_text(json.dumps({
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for folder in ('dra_mppi', 'tools', 'config') for p in (ROOT/folder).glob('*')
        if p.is_file()}, indent=2))

    diff = subprocess.run(['git', 'diff'], capture_output=True, text=True).stdout
    (args.output/'working.diff').write_text(diff)
    command = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], '--helper']
    log = (args.output/'fixture.log').open('w')
    child = subprocess.Popen(command, stdout=log, stderr=log)
    telemetry = []
    next_sample = time.monotonic()
    try:
        while node._cycle_index < args.warmup+args.cycles:
            if child.poll() is not None:
                raise RuntimeError('fixture exited; inspect fixture.log')
            executor.spin_once(timeout_sec=.5)
            if time.monotonic() >= next_sample:
                telemetry.append({'time': time.monotonic(), 'load': os.getloadavg(),
                    'rss_max_kib': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                    'threads': len(list(Path('/proc/self/task').iterdir())),
                    'thermal': {str(p): p.read_text().strip() for p in Path('/sys/class/thermal').glob('thermal_zone*/temp')},
                    'frequency_khz': {str(p): p.read_text().strip() for p in Path('/sys/devices/system/cpu').glob('cpu*/cpufreq/scaling_cur_freq')}})
                next_sample = time.monotonic()+10.

    finally:
        node.timer.cancel()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        child.send_signal(signal.SIGINT)
        child.wait(timeout=10.)
        log.close()
    rows = [json.loads(line) for line in (args.output/'cycles.jsonl').read_text().splitlines()]
    recorder = rows.pop()
    rows = rows[args.warmup:]
    (args.output/'telemetry.json').write_text(json.dumps(telemetry, indent=2))
    flat_rows = []
    for row in rows:
        flat = {key: value for key, value in row.items() if not isinstance(value, (dict, list))}
        flat.update(run_id=args.output.name, scenario=('detour' if args.detour else 'straight')+str(args.obstacles),
                    cycle_id=row['cycle'], total_ms=row['callback_ms'])
        flat.update({'stage_'+key+'_ms': value for key, value in (row.get('stage_ms') or {}).items()})
        flat_rows.append(flat)
    with (args.output/'timing.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=sorted(set().union(*(r.keys() for r in flat_rows))))
        writer.writeheader()
        writer.writerows(flat_rows)

    received = [json.loads(line) for line in (args.output/'receive.jsonl').read_text().splitlines()]
    received = [r for r in received if rows[0]['started'] <= r['received'] <= rows[-1]['finished']+.2]
    normal = [r for r in rows if r['status'] == 'normal']
    summary = {
        'scope': 'synthetic ROS DDS pilot' if args.cycles < 10000 else 'synthetic ROS DDS timing run',
        'cycles': len(rows), 'normal_optimizer_cycles': len(normal),
        'quantile_method': 'numpy percentile method=linear on raw cycles',
        'callback_over_200ms_count': sum(r['callback_ms'] > 200 for r in rows),
        'callback_over_200ms_ratio': sum(r['callback_ms'] > 200 for r in rows)/len(rows),
        'timeout_count': sum(r['status'] == 'timeout' for r in rows),
        'scheduled_response_ms': stats(r['scheduled_response_ms'] for r in rows),
        'backend': parameters['mppi.guide_evaluation'],
        'workers': parameters['dynamic_risk.guide_parallel_workers'],
        'frozen_nominal': args.frozen,

        'statuses': dict(Counter(r['status'] for r in rows)),
        'fallback_ratio': sum(r['status'] not in ('normal', 'terminal') for r in rows)/len(rows),
        'deadline_miss_ratio': sum(r['deadline_lateness_ms'] > 0 for r in rows)/len(rows),
        'skipped_periods': sum(r['skipped_periods'] for r in rows),
        'all_callback_ms': stats(r['callback_ms'] for r in rows),
        'normal_optimizer_ms': stats(r['optimizer_ms'] for r in normal),
        'start_delay_ms': stats(r['start_delay_ms'] for r in rows),
        'deadline_lateness_ms': stats(r['deadline_lateness_ms'] for r in rows),
        'publish_interval_ms': stats(r['publish_interval_ms'] for r in rows),
        'receive_interval_ms': stats(r['interval_ms'] for r in received),
        'receive_count_in_window': len(received), 'recorder': recorder,
        'covariance': args.covariance, 'risk_mode': parameters['dynamic_risk.mode'],
        'final_risk_mode': parameters.get('dynamic_risk.final_mode', 'inherit'),
        'max_obstacles': parameters.get('dynamic_risk.max_obstacles', 0),
        'warmup_cycles': args.warmup, 'obstacles': args.obstacles, 'modes': 2,
        'detour': args.detour, 'static_enabled': not args.static_off,
        'strategy': parameters['mppi.strategy'],
        'primary_K': parameters['mppi.guide_num_rollouts']
        if parameters['mppi.strategy'] == 'guide_conditioned' else parameters['mppi.num_rollouts'],
    }
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
