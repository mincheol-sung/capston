"""Export genuine consumed assignments, never reused coasting observations."""

from copy import deepcopy
from types import SimpleNamespace as NS

from amr_interfaces.msg import DetectedObject, ObservationProvenance
import numpy as np
from object_tracker.core.tracker import MultiObjectTracker
from object_tracker.observation_links import observation_links
from std_msgs.msg import Header


def detection(name='person', beam=3):
    obj = DetectedObject()
    obj.class_name = name
    obj.class_id = 0 if name == 'person' else -1
    obj.source = 2
    obj.confidence = .99
    observation = ObservationProvenance()
    observation.header.stamp.sec = 10
    observation.header.frame_id = 'laser_front'
    observation.sensor_id = '/scan_front'
    observation.observation_id = f'cluster-{beam}'
    observation.beam_indices = [beam, beam+1]
    obj.observations = [observation]
    return obj


def test_tracker_matches_use_stable_track_ids_and_current_measurement_indices():
    tracker = MultiObjectTracker()
    R = np.eye(2)*.01
    positions = [np.array([1., 0.]), np.array([4., 0.])]
    tracker.step(10., positions, [R, R], [detection(beam=1), detection(beam=8)])
    left, right = [t.id for t in tracker.tracks]
    assert tracker.last_measurement_matches == {left: 0, right: 1}
    # Reverse measurement order; it must not become a track-order assignment.
    tracker.step(10.1, positions[::-1], [R, R], [detection(beam=8), detection(beam=1)])
    assert tracker.last_measurement_matches == {left: 1, right: 0}
    tracker.step(10.1, positions, [R, R])  # repeated epoch
    assert tracker.last_measurement_matches == {}
    tracker.step(10.2, [], [])  # coast
    assert tracker.last_measurement_matches == {}


def test_export_preserves_sensor_epoch_indices_and_excludes_unmatched_tracks():
    a, b = detection(beam=1), detection(beam=8)
    original = deepcopy(b)
    trk = NS(id=42, lifecycle=1, motion=0, class_name='person', stamp=10.2,
             last_semantic_update=10.)
    coast = NS(id=43)
    header = Header()
    header.stamp.sec, header.stamp.nanosec, header.frame_id = 10, 200_000_000, 'base_link'
    output = observation_links([trk, coast], {42: 1}, [a, b], header)
    assert len(output.links) == 1
    link = output.links[0]
    assert link.track_id == 42 and link.observation == original.observations[0]
    assert output.header == header
    assert link.observation.header.stamp.sec == 10
    assert link.observation.header.stamp.nanosec == 0
    b.observations[0].beam_indices = [100]
    assert list(link.observation.beam_indices) == [8, 9]
    assert not observation_links([], {42: 1}, [a, b], header).links


def test_camera_only_is_not_a_lidar_mask_and_multiple_sensor_memberships_survive():
    obj = detection()
    camera = ObservationProvenance()
    camera.sensor_id = '/camera'
    rear = deepcopy(obj.observations[0])
    rear.sensor_id, rear.header.frame_id = '/scan_rear', 'laser_rear'
    obj.observations += [camera, rear]
    trk = NS(id=1, lifecycle=1, motion=0, class_name='person', stamp=10.,
             last_semantic_update=10.)
    links = observation_links([trk], {1: 0}, [obj], Header()).links
    assert [v.observation.sensor_id for v in links] == ['/scan_front', '/scan_rear']
    obj.observations = [camera]
    assert not observation_links([trk], {1: 0}, [obj], Header()).links
