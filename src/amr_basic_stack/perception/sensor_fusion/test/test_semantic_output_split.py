"""Keep anonymous LiDAR clusters out of the semantic tracker input."""
from types import SimpleNamespace

from amr_interfaces.msg import DetectedObject, DetectedObjectArray
import numpy as np

from sensor_fusion.camera_lidar_fusion_node import (
    CameraLidarFusionNode,
    SOURCE_CAMERA,
    SOURCE_FUSED,
    SOURCE_LIDAR,
)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _object(class_id, class_name, x, y, source):
    detected = DetectedObject()
    detected.header.frame_id = 'base_link'
    detected.class_id = class_id
    detected.class_name = class_name
    detected.confidence = 0.9
    detected.position.x = x
    detected.position.y = y
    detected.source = source
    return detected


def _array(*objects):
    message = DetectedObjectArray()
    message.header.frame_id = 'base_link'
    message.objects = list(objects)
    return message


def _node(forward_unmatched=False):
    semantic = _Publisher()
    tracking = _Publisher()
    obstacles = _Publisher()
    node = SimpleNamespace(
        get_logger=lambda: SimpleNamespace(warning=lambda *_args, **_kwargs: None),
        _lidar_xy=lambda obj: np.array([obj.position.x, obj.position.y]),
        cam_model=SimpleNamespace(cov=lambda *_args: np.eye(2) * 0.01),
        lid_model=SimpleNamespace(cov=lambda *_args: np.eye(2) * 0.01),
        _gate_cov=lambda *_args: np.zeros((2, 2)),
        gate=9.21,
        forward_unmatched_lidar=forward_unmatched,
        pub=semantic,
        tracking_pub=tracking,
        lidar_obstacles_pub=obstacles,
    )
    node._fuse = lambda *args: CameraLidarFusionNode._fuse(node, *args)
    node._passthrough = lambda *args: CameraLidarFusionNode._passthrough(node, *args)
    return node, semantic, tracking, obstacles


def test_unmatched_lidar_is_published_only_as_geometric_obstacle():
    node, semantic, tracking, obstacles = _node()
    camera = _array(_object(0, 'person', 1.0, 0.0, SOURCE_CAMERA))
    lidar = _array(
        _object(-1, 'lidar_cluster', 1.0, 0.0, SOURCE_LIDAR),
        _object(-1, 'lidar_cluster', 3.0, 1.0, SOURCE_LIDAR),
    )

    CameraLidarFusionNode._on_pair(node, camera, lidar)

    assert [obj.source for obj in semantic.messages[0].objects] == [SOURCE_FUSED]
    assert len(obstacles.messages[0].objects) == 1
    assert obstacles.messages[0].objects[0].source == SOURCE_LIDAR
    assert obstacles.messages[0].header.stamp == lidar.header.stamp
    assert [obj.source for obj in tracking.messages[0].objects] == [
        SOURCE_FUSED, SOURCE_LIDAR]


def test_camera_only_detection_stays_in_semantic_output():
    node, semantic, tracking, obstacles = _node()
    camera = _array(_object(0, 'person', 1.0, 0.0, SOURCE_CAMERA))
    lidar = _array(_object(-1, 'lidar_cluster', 4.0, 2.0, SOURCE_LIDAR))

    CameraLidarFusionNode._on_pair(node, camera, lidar)

    assert [obj.source for obj in semantic.messages[0].objects] == [SOURCE_CAMERA]
    assert len(obstacles.messages[0].objects) == 1
    assert [obj.source for obj in tracking.messages[0].objects] == [
        SOURCE_CAMERA, SOURCE_LIDAR]


def test_legacy_parameter_can_forward_unmatched_lidar():
    node, semantic, tracking, obstacles = _node(forward_unmatched=True)
    camera = _array()
    lidar = _array(_object(-1, 'lidar_cluster', 2.0, 0.0, SOURCE_LIDAR))

    CameraLidarFusionNode._on_pair(node, camera, lidar)

    assert [obj.source for obj in semantic.messages[0].objects] == [SOURCE_LIDAR]
    assert len(obstacles.messages[0].objects) == 1
    assert [obj.source for obj in tracking.messages[0].objects] == [SOURCE_LIDAR]
