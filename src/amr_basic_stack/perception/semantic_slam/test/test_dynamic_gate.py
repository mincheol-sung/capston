"""Dynamic-track exclusion must compare observations at their capture time."""

from types import SimpleNamespace as NS

import numpy as np

from semantic_slam.semantic_slam_node import SemanticSlamNode


def track(stamp, x, vx, pose_var=0.01, velocity_var=0.01, class_id=0, track_id=1):
    return (stamp, class_id, track_id, np.array([x, 0.0]), np.array([vx, 0.0]),
            np.eye(2) * pose_var, np.zeros((2, 2)), np.eye(2) * velocity_var)


def node(tracks):
    stub = NS(dynamic_tracks=tracks)
    stub._dynamic_predictions = lambda *a: SemanticSlamNode._dynamic_predictions(stub, *a)
    return stub


def dynamic(tracks, stamp, x, class_id=0):
    return SemanticSlamNode._dynamic_at(node(tracks), stamp, class_id, np.array([x, 0.0]))


def inside(tracks, stamp, center, class_id=0, half=0.225):
    rng = np.random.default_rng(0)
    points = np.asarray(center) + rng.uniform(-half, half, size=(200, 3))
    return SemanticSlamNode._dynamic_inside(node(tracks), stamp, class_id, points)


def test_delayed_observation_matches_track_predicted_to_capture_time():
    # 1.0 m/s person; the observation was captured 0.8 s before the latest track.
    tracks = [track(10.0, 0.0, 1.0)]
    assert dynamic(tracks, 9.2, -0.8)
    assert not dynamic(tracks, 9.2, 0.8)


def test_nearest_sample_of_each_track_is_used():
    tracks = [track(9.0, -1.0, 1.0), track(10.0, 0.0, 1.0)]
    assert dynamic(tracks, 9.05, -0.95)


def test_gate_grows_with_prediction_uncertainty_but_is_bounded():
    assert not dynamic([track(10.0, 0.0, 0.0)], 9.2, 0.9)
    assert dynamic([track(10.0, 0.0, 0.0, velocity_var=0.5)], 9.2, 0.9)
    assert not dynamic([track(10.0, 0.0, 0.0, velocity_var=100.0)], 9.2, 2.1)


def test_class_and_time_window_are_respected():
    tracks = [track(10.0, 0.0, 0.0, class_id=3)]
    assert not dynamic(tracks, 10.0, 0.0, class_id=4)
    assert dynamic([track(10.0, 0.0, 0.0, class_id=-1)], 10.0, 0.0, class_id=4)
    assert not dynamic(tracks, 8.5, 0.0, class_id=3)


def test_track_passing_beside_object_does_not_claim_it_moved():
    # LiDAR-only person 0.6 m beside a 0.45 m chair: inside the exclusion gate,
    # outside the chair footprint. The observation is dropped, the chair kept.
    tracks = [track(10.0, 3.0, 1.0, class_id=-1)]
    chair = [3.0, -0.6, 0.4]
    assert SemanticSlamNode._dynamic_at(node(tracks), 10.0, 56, np.array(chair[:2]))
    assert not inside(tracks, 10.0, chair, class_id=56)


def test_track_on_object_footprint_claims_it_moved():
    # A dragged chair seen only by LiDAR: the track centroid lies on the chair.
    tracks = [track(10.0, 3.05, 0.5, class_id=-1)]
    assert inside(tracks, 10.0, [3.0, 0.0, 0.4], class_id=56)
    assert inside([track(9.8, 2.95, 0.5, class_id=56)], 10.0, [3.0, 0.0, 0.4], class_id=56)
