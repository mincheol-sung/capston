"""Exercise actual YOLO postprocessing and capture-time TF lookup contracts."""

from types import SimpleNamespace as NS
from collections import Counter

from builtin_interfaces.msg import Time

import numpy as np
import pytest
from sensor_msgs.msg import Image

from sensor_fusion.rgbd_detection_node import RgbdDetectionNode
from sensor_fusion.rgbd_detection_node import YoloOpenVINODetector


class Compiled:
    """Fake raw OpenVINO output tensors, not postprocessed detections."""

    def __call__(self, _inputs):
        prediction = np.zeros((1, 116, 2), dtype=np.float32)
        prediction[0, :4, 0] = [4, 4, 4, 4]
        prediction[0, 4 + 56, 0] = 0.9
        # A mask coefficient higher than every class score must not become a class.
        prediction[0, 84, 0] = 10
        return {'pred': prediction, 'proto': np.ones((1, 32, 8, 8), dtype=np.float32)}


def test_segmentation_coefficients_are_not_classes():
    detector = YoloOpenVINODetector.__new__(YoloOpenVINODetector)
    detector.compiled, detector.output_layer = Compiled(), 'pred'
    detector.input_h, detector.input_w = 8, 8
    detector.names = {i: ('chair' if i == 56 else str(i)) for i in range(80)}
    detector.conf, detector.iou, detector.class_filter = 0.4, 0.45, None
    detector.mask_classes, detector.max_masks = {56, 60}, 8
    detections = detector.detect(np.zeros((4, 8, 3), dtype=np.uint8))
    assert len(detections) == 1 and detections[0]['class_id'] == 56
    assert detections[0]['mask'].shape == (4, 8)
    assert detections[0]['mask'].any()


def test_lookup_uses_capture_time():
    calls = []
    node = RgbdDetectionNode.__new__(RgbdDetectionNode)
    node.target_frame = 'base_link'
    node.tf_buffer = NS(lookup_transform=lambda *args: calls.append(args) or 'transform')
    header = NS(frame_id='camera', stamp=Time(sec=42, nanosec=123))
    assert node._lookup_tf(header) == 'transform'
    assert calls[0][2].nanoseconds == 42_000_000_123


def rgbd_pair_node():
    node = RgbdDetectionNode.__new__(RgbdDetectionNode)
    node.max_age, node.rgbd_max_delta = .75, .025
    node.timing_mode, node.unknown_policy = 'unknown', 'snapshot'
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=1_100_000_000))
    accepted = []
    node.pending = NS(dropped=Counter(), add=lambda *args: accepted.append(args))
    node._drain_pairs = lambda: None
    color, depth = Image(), Image()
    for image in (color, depth):
        image.header.stamp.sec = 1
        image.header.frame_id = 'camera_color_optical_frame'
    return node, color, depth, accepted


@pytest.mark.parametrize('difference', ['stamp', 'frame', 'zero'])
def test_rgbd_nearby_but_unaligned_captures_do_not_reach_inference(difference):
    node, color, depth, accepted = rgbd_pair_node()
    if difference == 'stamp':
        color.header.stamp.nanosec = 1_000_000  # Inside both ATS and 25ms limits.
    elif difference == 'frame':
        color.header.frame_id = 'another_optical_frame'
    else:
        depth.header.stamp.sec = color.header.stamp.sec = 0
    node._on_pair(color, depth)
    assert not accepted and sum(node.pending.dropped.values()) == 1


def test_rgbd_unknown_timing_policy_is_explicit():
    node, color, depth, accepted = rgbd_pair_node()
    node.unknown_policy = 'drop'
    node._on_pair(color, depth)
    assert not accepted and node.pending.dropped['timing_unknown_drop'] == 1
    node.unknown_policy = 'snapshot'
    node._on_pair(color, depth)
    assert len(accepted) == 1 and accepted[0][0] == 1_000_000_000


@pytest.mark.parametrize('mode,status', [('unknown', 0), ('capture', 1)])
def test_camera_provenance_does_not_claim_verified_exposures_from_equal_stamps(mode, status):
    from geometry_msgs.msg import TransformStamped
    node, color, depth, _ = rgbd_pair_node()
    node.timing_mode = mode
    color.encoding, depth.encoding = 'bgr8', '16UC1'
    node.K = np.array([[100., 0., 1.], [0., 100., 1.], [0., 0., 1.]])
    node.info_shape, node.info_frame = (3, 3), depth.header.frame_id
    node.inference_hz, node.last_inference, node.depth_scale = 0., -float('inf'), .001
    node.bridge = NS(imgmsg_to_cv2=lambda image, **_: (
        np.zeros((3, 3, 3), dtype=np.uint8) if image is color
        else np.full((3, 3), 2000, dtype=np.uint16)))
    node.detector = NS(detect=lambda _: [dict(
        class_id=0, class_name='person', confidence=1., cx=1., cy=1., w=2., h=2.)])
    node.target_frame, node.cam_pos_method, node.sample_win = 'base_link', 'center_patch', 0
    transform = TransformStamped()
    transform.transform.rotation.w = 1.
    node.tf_buffer = NS(lookup_transform=lambda *_: transform)
    node.debug_center_box = node.semantic_enabled = False
    node.depth_topic, node.color_topic = '/depth', '/color'
    output = []
    node.pub = NS(publish=output.append)
    node._process_pair(color, depth)
    assert len(output) == 1 and len(output[0].objects) == 1
    obj = output[0].objects[0]
    assert obj.header == output[0].header and obj.header.stamp == depth.header.stamp
    assert [obs.header for obs in obj.observations] == [depth.header, color.header]
    assert [obs.timing_status for obs in obj.observations] == [status, status]
    assert obj.position.z == 2.


def test_rgbd_invalid_header_never_queries_latest_tf():
    node = RgbdDetectionNode.__new__(RgbdDetectionNode)
    assert node._lookup_tf(NS(frame_id='camera', stamp=Time())) is None
    assert node._lookup_tf(NS(frame_id='', stamp=Time(sec=1))) is None
