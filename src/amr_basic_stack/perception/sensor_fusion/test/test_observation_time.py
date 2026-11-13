"""Numerical motion truth, real tf2 interpolation and outage/identity contracts."""
import copy
import math
from pathlib import Path
import time
from types import SimpleNamespace as NS

from amr_interfaces.msg import DetectedObject, DetectedObjectArray, ObservationProvenance
from amr_laser_tools.scan_sector_filter import filter_scan
from geometry_msgs.msg import TransformStamped
import numpy as np
from object_tracker.core.pending import PendingMeasurements
import pytest
import rclpy
from scipy.spatial.transform import Rotation
from sensor_fusion.camera_lidar_fusion_node import CameraLidarFusionNode
from sensor_fusion.lidar_cluster_node import LidarClusterNode
from sensor_fusion.observation_time import (
    beam_offsets, deskew, FusionQueue, rigid, temporal_variance)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer


def stamp(seconds):
    return rclpy.time.Time(nanoseconds=round(seconds*1e9)).to_msg()


def tf(parent, child, seconds=1., xyz=(0., 0., 0.), yaw=0.):
    out = TransformStamped()
    out.header.frame_id, out.child_frame_id = parent, child
    out.header.stamp = stamp(seconds)
    out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = xyz
    q = Rotation.from_euler('z', yaw).as_quat()
    out.transform.rotation.x, out.transform.rotation.y = float(q[0]), float(q[1])
    out.transform.rotation.z, out.transform.rotation.w = float(q[2]), float(q[3])
    return out


def observation(seconds=1., sensor='/scan_front', offsets=(0., .1), indices=(0, 1)):
    obs = ObservationProvenance()
    obs.header.frame_id, obs.header.stamp = 'laser_front', stamp(seconds)
    obs.sensor_id = sensor
    obs.observation_id = f'{sensor}|{seconds}|{indices}'
    obs.timing_status = 1
    obs.beam_indices, obs.beam_time_offsets = list(indices), list(offsets)
    return obs


def array(seconds=1., source=1, x=2., sensor='/scan_front'):
    arr = DetectedObjectArray()
    arr.header.frame_id, arr.header.stamp = 'base_link', stamp(seconds)
    obj = DetectedObject()
    obj.header = arr.header
    obj.position.x, obj.position.y, obj.position.z = x, .0, .2
    obj.source = source
    obj.class_id, obj.class_name = (-1, 'lidar_cluster') if source else (0, 'person')
    obj.confidence = 1.
    obj.position_covariance = [.01, .002, .002, .02]
    obj.observations = [observation(seconds, sensor, (0.,), (0,))]
    obj.ego_motion_compensated = source == 1
    arr.observations = copy.deepcopy(obj.observations)
    arr.objects = [obj]
    return arr


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def cluster():
    node = LidarClusterNode()
    node.tf_buffer = Buffer()
    node.tf_buffer.set_transform_static(tf('base_link', 'laser_front'), 'test')
    node.tf_buffer.set_transform_static(tf('odom', 'base_link'), 'test')
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_200_000_000))
    node.timing_modes = ['ros', 'ros']
    node.min_points, node.min_width, node.max_width = 1, 0., 20.
    node.pub = Publisher()
    yield node
    node.destroy_node()


@pytest.fixture
def fusion():
    node = CameraLidarFusionNode()
    node.tf_buffer = Buffer()
    node.tf_buffer.set_transform_static(tf('odom', 'base_link'), 'test')
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_500_000_000))
    node.pub, node.tracking_pub, node.lidar_obstacles_pub = Publisher(), Publisher(), Publisher()
    # Isolate motion from the unchanged C2 calibration in numerical tests.
    node.lidar_yaw = 0.
    node.offset.r_coef, node.offset.t_const = 0., 0.
    yield node
    node.destroy_node()


def test_valid_invalid_and_reversed_driver_timing():
    assert np.allclose(beam_offsets(4, .01, 'ros'), [0., .01, .02, .03])
    assert np.allclose(beam_offsets(4, .01, 'reversed'), [.03, .02, .01, 0.])
    for increment in (0., -.1, float('nan'), float('inf'), 1.):
        assert beam_offsets(4, increment, 'ros') is None
    assert beam_offsets(4, .01, 'unknown') is None
    with pytest.raises(ValueError):
        beam_offsets(4, .01, 'invented')


def test_indices_survive_invalid_beams_sector_mask_and_clusters(cluster):
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'laser_front', stamp(1.)
    scan.angle_min, scan.angle_increment = -2., .5
    scan.range_min, scan.range_max, scan.time_increment = .1, 10., .01
    scan.ranges = [2., 2., float('inf'), 2., float('nan'), 2., 2., 2., 2.]
    masked = filter_scan(scan, (0., 0., 0.), (-math.pi/2, math.pi/2))
    result = cluster._prepare(masked, 0)
    cluster._publish(result, masked, 0)
    objects = cluster.pub.messages[0].objects
    indices = sorted(i for obj in objects for i in obj.observations[0].beam_indices)
    assert indices == [1, 3, 5, 6, 7]
    for obj in objects:
        obs = obj.observations[0]
        assert obs.header == scan.header
        assert obs.sensor_id == '/scan_front'
        assert np.allclose(obs.beam_time_offsets, np.array(obs.beam_indices)*scan.time_increment)
        assert obs.observation_id and obj.ego_motion_compensated
        assert obj.header == cluster.pub.messages[0].header
    assert masked.header == scan.header and len(masked.ranges) == len(scan.ranges)


def test_wraparound_membership_preserves_native_indices(cluster):
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'laser_front', stamp(1.)
    scan.angle_min, scan.angle_increment = -math.pi, math.pi/180
    scan.range_min, scan.range_max, scan.time_increment = .1, 10., .0002
    scan.ranges = [float('inf')]*360
    for i in [0, 1, 358, 359]:
        scan.ranges[i] = 2.
    cluster.sector_min[0], cluster.sector_max[0] = -180., 180.
    cluster._publish(cluster._prepare(scan, 0), scan, 0)
    objects = cluster.pub.messages[0].objects
    assert len(objects) == 1
    assert list(objects[0].observations[0].beam_indices) == [0, 1, 358, 359]


def test_unknown_timing_never_claims_deskew(cluster):
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'laser_front', stamp(1.)
    scan.angle_increment, scan.range_min, scan.range_max = .01, .1, 10.
    scan.scan_time, scan.time_increment, scan.ranges = .1, 0., [2., 2., 2.]
    cluster._publish(cluster._prepare(scan, 0), scan, 0)
    obj = cluster.pub.messages[0].objects[0]
    assert not obj.ego_motion_compensated
    assert obj.observations[0].timing_status == 0
    assert not obj.observations[0].beam_time_offsets
    cluster.unknown_policy = 'drop'
    assert cluster._prepare(scan, 0)[0] is None


@pytest.mark.parametrize('velocity,omega,lever', [
    (0., 0., 0.), (.8, 0., .3), (.6, 1.4, .4), (0., -2., .4)])
def test_stationary_world_points_deskew_to_truth(velocity, omega, lever):
    times = np.linspace(0., .1, 2000)
    world = np.column_stack((np.full(len(times), 4.), np.linspace(-.5, .5, len(times)),
                             np.full(len(times), .2)))

    def pose(t):
        r = Rotation.from_euler('z', omega*t).as_matrix()
        return r, np.array([velocity*t, 0., 0.])+r@np.array([lever, 0., 0.])
    rt = Rotation.from_euler('z', omega*.1).as_matrix()
    tt = np.array([velocity*.1, 0., 0.])
    raw = np.stack([pose(t)[0].T@(p-pose(t)[1]) for t, p in zip(times, world)])
    calls = []

    def lookup(t):
        calls.append(t)
        r, translation = pose(t)
        return rt.T@r, rt.T@(translation-tt)
    started = time.perf_counter()
    actual = deskew(raw, times, lookup, .01)
    elapsed = time.perf_counter()-started
    expected = (world-tt)@rt
    error = np.max(np.linalg.norm(actual-expected, axis=1))
    assert error < 2.1e-5  # <=20um chord error at 2rad/s, 0.4m lever, 10ms knots
    assert len(calls) == 11
    if velocity == omega == 0:
        assert np.allclose(actual, raw+[lever, 0., 0.])
    else:
        naive = raw@lookup(.1)[0].T+lookup(.1)[1]
        assert np.max(np.linalg.norm(naive-expected, axis=1)) > .01
    print(f'2000 beams: {elapsed*1000:.3f}ms, max error {error:.9f}m, 11 TF knots')


def test_real_tf_full_accounts_for_rotation_translation_and_mounting():
    buffer = Buffer()
    buffer.set_transform_static(tf('base_link', 'laser_front', xyz=(.4, -.1, .2)), 'test')
    buffer.set_transform(tf('odom', 'base_link', 1.), 'test')
    buffer.set_transform(tf('odom', 'base_link', 1.1, (1., .2, 0.), .5), 'test')
    r, t = rigid(buffer.lookup_transform_full('base_link', rclpy.time.Time(seconds=1.1),
                 'laser_front', rclpy.time.Time(seconds=1.), 'odom'))
    expected = Rotation.from_euler('z', -.5).apply(np.array([2.4, -.1, .2])-[1., .2, 0.])
    assert np.allclose(r@np.array([2., 0., 0.])+t, expected)


def test_object_motion_is_not_removed_and_uncertainty_uses_seconds():
    times = np.linspace(0., .1, 100)
    raw = np.column_stack((4.+2.*times, times*0., times*0.))
    corrected = deskew(raw, times, lambda _: (np.eye(3), np.zeros(3)))
    assert np.ptp(corrected[:, 0]) == pytest.approx(.2)
    obj = array().objects[0]
    obj.observations = [observation(1., offsets=(0., .1))]
    assert temporal_variance(obj, 1_100_000_000, 2., .15) == pytest.approx(.04)
    obj.observations[0].timing_status = 0
    assert temporal_variance(obj, 1_100_000_000, 2., .15) == pytest.approx(.0625)


def test_fusion_aligns_before_association_and_preserves_provenance(fusion):
    fusion.tf_buffer = Buffer()
    fusion.tf_buffer.set_transform(tf('odom', 'base_link', 1.), 'test')
    fusion.tf_buffer.set_transform(tf('odom', 'base_link', 1.1, (.1, 0., 0.)), 'test')
    cam, lid = array(1.05, 0, 1.95, 'depth'), array(1., 1, 2.)
    # Sparse native indices must not become positions in the compact cluster.
    lid.objects[0].observations[0].beam_indices = [2, 5, 9]
    lid.objects[0].observations[0].beam_time_offsets = [0., .02, .04]
    originals = copy.deepcopy(cam.objects[0].observations + lid.objects[0].observations)
    aligned_c = fusion._align(cam, 1_050_000_000, True)
    aligned_l = fusion._align(lid, 1_050_000_000, False)
    assert aligned_c.objects[0].position.x == pytest.approx(aligned_l.objects[0].position.x)
    fusion._on_pair(aligned_c, aligned_l)
    output = fusion.tracking_pub.messages[0]
    assert len(output.objects) == 1 and output.objects[0].source == 2
    assert output.objects[0].header == output.header
    assert output.objects[0].observations[1].header.stamp == stamp(1.)
    assert output.objects[0].observations == originals
    assert cam.objects[0].observations + lid.objects[0].observations == originals
    assert not fusion.lidar_obstacles_pub.messages[0].objects
    covariance = np.array(output.objects[0].position_covariance).reshape(2, 2)
    assert np.allclose(covariance, covariance.T)
    assert np.linalg.eigvalsh(covariance).min() > 0


def test_covariance_rotates_and_temporal_variance_reaches_fusion(fusion):
    fusion.tf_buffer = Buffer()
    fusion.tf_buffer.set_transform(tf('odom', 'base_link', 1.), 'test')
    fusion.tf_buffer.set_transform(tf('odom', 'base_link', 1.1, yaw=math.pi/2), 'test')
    msg = array(1., 0, sensor='depth')
    actual = fusion._align(msg, 1_100_000_000, True).objects[0]
    c = np.array([[0., 1.], [-1., 0.]])
    expected = c@np.array([[.01, .002], [.002, .02]])@c.T+np.eye(2)*.01
    assert np.allclose(np.array(actual.position_covariance).reshape(2, 2), expected)
    # Temporal R is carried into the result, not only the gate.
    cam, lid = array(1., 0, sensor='depth'), array(1.)
    a = fusion._align(cam, 1_100_000_000, True)
    b = fusion._align(lid, 1_100_000_000, False)
    fusion._on_pair(a, b)
    fused = np.array(fusion.pub.messages[-1].objects[0].position_covariance).reshape(2, 2)
    assert np.trace(fused) > .025


def test_front_calibration_never_applies_to_rear(fusion):
    fusion.lidar_yaw, fusion.offset.r_coef, fusion.offset.t_const = -.043670, -.01098, .013
    front, rear = array().objects[0], array(sensor='/scan_rear').objects[0]
    assert np.allclose(fusion._lidar_xy(rear), [2., 0.])
    assert not np.allclose(fusion._lidar_xy(front), [2., 0.])


def test_tf_delay_missing_duplicate_and_out_of_order_queue():
    q = FusionQueue()
    q.add('front', array(1.1), 0., 1_200_000_000)
    q.add('front', array(1.), .01, 1_200_000_000)
    q.add('front', array(1.), .02, 1_200_000_000)
    seen = []
    q.drain(.12, 1_200_000_000, lambda *_: False)
    assert q.watermark == -1 and q.counts['tf_wait'] == 1
    q.drain(.13, 1_200_000_000, lambda batch, ref: seen.append(ref) or True)
    assert seen == [1_000_000_000, 1_100_000_000]
    assert q.counts['duplicate'] == 1 and q.counts['reordered'] == 1
    q.add('front', array(1.), .14, 1_200_000_000)
    assert q.counts['late_or_duplicate'] == 1
    q.add('rear', array(1.2), .2, 1_300_000_000)
    q.drain(.56, 1_300_000_000, lambda *_: False)
    assert q.counts['tf_timeout'] == 1 and not q.entries


def test_camera_outage_recovery_consumes_each_lidar_only_once(fusion):
    q = FusionQueue()
    q.add('front', array(1.), 0., 1_010_000_000)
    q.drain(.11, 1_120_000_000, fusion._consume)
    assert len(fusion.tracking_pub.messages) == 1
    assert fusion.tracking_pub.messages[-1].objects[0].source == 1
    q.add('camera', array(1., 0, sensor='depth'), .12, 1_120_000_000)
    q.add('front', array(1.1), .12, 1_120_000_000)
    q.add('camera', array(1.12, 0, sensor='depth'), .13, 1_130_000_000)
    q.drain(.24, 1_250_000_000, fusion._consume)
    assert len(fusion.tracking_pub.messages) == 2
    assert fusion.tracking_pub.messages[-1].objects[0].source == 2
    assert q.counts['late_or_duplicate'] == 1
    ids = [o.observation_id for m in fusion.tracking_pub.messages
           for obj in m.objects for o in obj.observations if o.sensor_id == '/scan_front']
    assert len(ids) == len(set(ids)) == 2


def test_camera_outage_keeps_one_tracker_identity(fusion):
    from object_tracker.object_tracker_node import ObjectTrackerNode
    tracker = ObjectTrackerNode()
    tracker.tf_buffer = fusion.tf_buffer
    tracker.publish_tentative = True
    tracker.pub, tracker.modes_pub, tracker.marker_pub = Publisher(), Publisher(), None
    q = FusionQueue()
    track_ids = []
    try:
        for index in range(8):
            epoch = 1.+index*.1
            ros_ns = round((epoch+.12)*1e9)
            fusion.get_clock = tracker.get_clock = lambda: NS(
                now=lambda: NS(nanoseconds=ros_ns))
            q.add('front', array(epoch), index*.2, ros_ns)
            if index < 3 or index >= 6:
                q.add('camera', array(epoch, 0, sensor='depth'), index*.2, ros_ns)
            q.drain(index*.2+.11, ros_ns, fusion._consume)
            msg = fusion.tracking_pub.messages[-1]
            tracker._on_detections(msg)
            assert len(tracker.tracker.tracks) == 1
            track_ids.append(tracker.tracker.tracks[0].id)
        assert len(set(track_ids)) == 1
        # A replay at its old epoch cannot update or create another track.
        tracker._on_detections(fusion.tracking_pub.messages[-1])
        assert tracker.pending.dropped['duplicate_observation'] == 1
        assert len(tracker.tracker.tracks) == 1
    finally:
        tracker.destroy_node()


def test_cluster_waits_for_real_tf_then_drops_missing_transform(cluster):
    scan = LaserScan()
    scan.header.frame_id, scan.header.stamp = 'laser_front', stamp(1.)
    scan.angle_increment, scan.range_min, scan.range_max = .01, .1, 10.
    scan.ranges, scan.time_increment = [2., 2., 2.], .01
    cluster.tf_buffer = Buffer()
    queue = PendingMeasurements(.25, 2)
    queue.add(1_000_000_000, scan, 0.)
    queue.drain(.01, lambda m: cluster._prepare(m, 0),
                lambda m, result: cluster._publish(result, m, 0))
    assert not cluster.pub.messages
    cluster.tf_buffer.set_transform_static(tf('base_link', 'laser_front'), 'test')
    cluster.tf_buffer.set_transform_static(tf('odom', 'base_link'), 'test')
    queue.drain(.02, lambda m: cluster._prepare(m, 0),
                lambda m, result: cluster._publish(result, m, 0))
    assert len(cluster.pub.messages) == 1
    queue.add(1_100_000_000, scan, .03)
    queue.drain(.3, lambda _: None, lambda *_: pytest.fail('missing TF consumed'))
    assert queue.dropped['tf_expired'] == 1


def test_queue_capacity_age_and_epoch_mismatch(fusion):
    q = FusionQueue(capacity=2)
    for i in range(3):
        q.add('front', array(1.+i*.1), 0., 1_300_000_000)
    assert len(q.entries) == 2 and q.counts['overflow'] == 1
    q.drain(1., 2_000_000_000, lambda *_: pytest.fail('stale consumed'))
    assert not q.entries and q.counts['stale'] == 2
    cam, lid = array(1., 0), array(1.1)
    fusion._on_pair(cam, lid)
    assert not fusion.pub.messages


def test_new_coordinate_epoch_does_not_make_old_capture_fresh():
    msg = array(1.)
    msg.header.stamp = stamp(1.7)
    q = FusionQueue(max_age=.75)
    q.add('front', msg, 0., 1_800_000_000)
    assert not q.entries and q.counts['stale'] == 1


def test_fusion_checks_individual_capture_age_even_without_array_records():
    msg = array(1.)
    msg.observations = []
    msg.header.stamp = msg.objects[0].header.stamp = stamp(1.7)
    q = FusionQueue(max_age=.75)
    q.add('front', msg, 0., 1_800_000_000)
    assert not q.entries and q.counts['stale'] == 1


@pytest.mark.parametrize('source,reference', [(0., 1.), (1., 0.), (1.1, 1.)])
def test_fusion_never_uses_latest_tf_or_a_reference_before_capture(fusion, source, reference):
    with pytest.raises(ValueError):
        fusion._align(array(source), round(reference*1e9), False)


def test_stationary_target_stays_stationary_after_robot_translation_and_rotation(fusion):
    from object_tracker.object_tracker_node import ObjectTrackerNode
    from sensor_fusion.observation_time import Entry

    def pose_at(t):
        rotation = Rotation.from_euler('z', .8*(t-1.)).as_matrix()
        return rotation, np.array([.4*(t-1.), -.2*(t-1.), 0.])

    fusion.tf_buffer = Buffer()
    for i in range(16):
        t = 1.+i*.05
        _, translation = pose_at(t)
        fusion.tf_buffer.set_transform(tf('odom', 'base_link', t, translation, .8*(t-1.)), 'test')
    tracker = ObjectTrackerNode()
    tracker.tf_buffer = fusion.tf_buffer
    tracker.publish_tentative = True
    tracker.pub, tracker.modes_pub = Publisher(), Publisher()
    tracker.observations_pub, tracker.marker_pub = Publisher(), None
    world = np.array([4., 1., .2])
    try:
        for i in range(6):
            lid_time, cam_time = 1.+i*.1, 1.05+i*.1
            cam, lid = array(cam_time, 0, sensor='depth'), array(lid_time)
            cam.objects[0].observations[0].beam_indices = []
            cam.objects[0].observations[0].beam_time_offsets = []
            for msg, t in ((cam, cam_time), (lid, lid_time)):
                rotation, translation = pose_at(t)
                point = rotation.T @ (world-translation)
                msg.objects[0].position.x, msg.objects[0].position.y, msg.objects[0].position.z = map(float, point)
            assert abs(cam.objects[0].position.y-lid.objects[0].position.y) > .01
            now_ns = round((cam_time+.1)*1e9)
            fusion.get_clock = tracker.get_clock = lambda: NS(now=lambda: NS(nanoseconds=now_ns))
            original = copy.deepcopy(lid.objects[0].observations[0])
            assert fusion._consume([Entry(round(lid_time*1e9), 'front', lid, time.monotonic()),
                                    Entry(round(cam_time*1e9), 'camera', cam, time.monotonic())],
                                   round(cam_time*1e9))
            fused = fusion.tracking_pub.messages[-1]
            assert len(fused.objects) == 1 and fused.objects[0].source == 2
            assert fused.objects[0].observations[-1] == original
            tracker._on_detections(fused)
            output = tracker.pub.messages[-1]
            assert len(output.objects) == 1 and output.header.frame_id == 'odom'
            obj = output.objects[0]
            assert obj.header == output.header and obj.header.stamp == stamp(cam_time)
            np.testing.assert_allclose([obj.pose.pose.position.x, obj.pose.pose.position.y], world[:2], atol=1e-6)
            np.testing.assert_allclose([obj.velocity.twist.linear.x, obj.velocity.twist.linear.y], 0., atol=1e-6)
            assert tracker.observations_pub.messages[-1].links[0].observation == original
        assert len({msg.objects[0].track_id for msg in tracker.pub.messages}) == 1
    finally:
        tracker.destroy_node()


def test_launch_connects_native_scans_and_timing_config():
    root = Path(__file__).resolve().parents[3]
    config_path = root/'perception/amr_perception_bringup/config/observation_timing.yaml'
    config = config_path.read_text()
    launch_path = root/'perception/amr_perception_bringup/launch/perception_stack.launch.py'
    launch = launch_path.read_text()
    system = (root/'stack_master/launch/amr_system.launch.py').read_text()
    assert 'scan_topics: [/scan_front, /scan_rear]' in config
    assert 'timing_modes: [unknown, unknown]' in config
    assert launch.count('parameters=[sim, timing_config,') == 3
    assert "'detections_topic': '/detections_tracking'" in launch
    argument = '"observation_timing_config": LaunchConfiguration("observation_timing_config")'
    assert argument in system
