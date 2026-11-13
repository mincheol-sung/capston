"""Reject observations in incompatible coordinate frames before association."""
from types import SimpleNamespace

from amr_interfaces.msg import DetectedObjectArray

from sensor_fusion.camera_lidar_fusion_node import CameraLidarFusionNode


def test_incompatible_frames_are_not_fused_or_relabelled():
    """A LiDAR TF fallback must not silently become camera-frame geometry."""
    camera, lidar = DetectedObjectArray(), DetectedObjectArray()
    camera.header.frame_id = 'base_link'
    lidar.header.frame_id = 'base_scan'
    warnings = []
    node = SimpleNamespace(get_logger=lambda: SimpleNamespace(
        warning=lambda text, **kwargs: warnings.append(text)))
    CameraLidarFusionNode._on_pair(node, camera, lidar)
    assert len(warnings) == 1
