#!/usr/bin/env python3
"""Render multi-modal obstacle predictions as RViz MarkerArray messages."""
import math

from amr_interfaces.msg import (
    MotionModePrediction,
    MultiModalObjectPredictionArray,
)
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


MODE_STYLES = {
    MotionModePrediction.MODEL_CV: ('cv', (0.05, 0.70, 1.00)),
    MotionModePrediction.MODEL_CT: ('ct', (1.00, 0.35, 0.05)),
    MotionModePrediction.MODEL_STATIC: ('static', (0.20, 0.90, 0.30)),
}
MOTION_NAMES = {0: 'UNKNOWN', 1: 'STATIC', 2: 'DYNAMIC'}


def _alpha(weight, minimum=0.12, maximum=0.95):
    """Map a mode probability to a visible RViz alpha."""
    value = float(np.clip(weight, 0.0, 1.0))
    return minimum + (maximum - minimum) * value


def _color(rgb, weight):
    """Create one per-vertex color whose alpha represents mode probability."""
    return ColorRGBA(
        r=float(rgb[0]),
        g=float(rgb[1]),
        b=float(rgb[2]),
        a=_alpha(weight),
    )


def _point(x, y, z):
    """Create a finite planar visualization point."""
    return Point(x=float(x), y=float(y), z=float(z))


def _ellipse_segments(mean, covariance, scale, segment_count, z):
    """Return LINE_LIST vertices for a scaled 2-D covariance ellipse."""
    covariance = np.asarray(covariance, dtype=float).reshape(2, 2)
    covariance = 0.5 * (covariance + covariance.T)
    if not np.all(np.isfinite(covariance)):
        return []
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    if eigenvalues[0] < -1e-9:
        return []
    radii = float(scale) * np.sqrt(np.maximum(eigenvalues, 0.0))
    angles = np.linspace(0.0, 2.0 * math.pi, segment_count + 1)
    unit_circle = np.vstack((np.cos(angles), np.sin(angles)))
    xy = np.asarray(mean, dtype=float)[:, None] + (
        eigenvectors @ (radii[:, None] * unit_circle)
    )
    vertices = []
    for index in range(segment_count):
        vertices.append(_point(xy[0, index], xy[1, index], z))
        vertices.append(_point(xy[0, index + 1], xy[1, index + 1], z))
    return vertices


def _base_marker(header, namespace, marker_id, marker_type, lifetime):
    """Create a consistently initialized prediction marker."""
    marker = Marker()
    marker.header = header
    marker.ns = namespace
    marker.id = marker_id
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime = Duration(seconds=lifetime).to_msg()
    return marker


def build_prediction_markers(message, covariance_scale=2.4477,
                             ellipse_segments=24, covariance_stride=5,
                             lifetime=0.5, line_width=0.025,
                             point_diameter=0.06):
    """Convert one prediction array to paths, means, ellipses, and labels."""
    output = MarkerArray()
    clear = Marker()
    clear.header = message.header
    clear.action = Marker.DELETEALL
    output.markers.append(clear)

    segment_count = max(8, int(ellipse_segments))
    ellipse_stride = max(1, int(covariance_stride))
    for predicted in message.objects:
        marker_id = int(predicted.track_id % 2147483647)
        label_candidates = []

        for mode in predicted.modes:
            style = MODE_STYLES.get(int(mode.model_type))
            if style is None:
                continue
            mode_name, rgb = style
            count = min(len(predicted.time_offsets), len(mode.points))
            valid = []
            for point_index, prediction in enumerate(mode.points[:count]):
                mean = np.array(
                    [prediction.mean_x, prediction.mean_y], dtype=float)
                if np.all(np.isfinite(mean)):
                    valid.append((point_index, prediction, mean))
            if not valid:
                continue

            z = (0.08 if mode.model_type == MotionModePrediction.MODEL_CV
                 else 0.11)
            path = _base_marker(
                message.header, f'prediction_{mode_name}_path', marker_id,
                Marker.LINE_STRIP, lifetime)
            path.scale.x = float(line_width)
            path.points = [_point(mean[0], mean[1], z)
                           for _, _, mean in valid]
            path.colors = [_color(rgb, prediction.weight)
                           for _, prediction, _ in valid]
            output.markers.append(path)

            means = _base_marker(
                message.header, f'prediction_{mode_name}_means', marker_id,
                Marker.SPHERE_LIST, lifetime)
            means.scale.x = means.scale.y = means.scale.z = float(point_diameter)
            means.points = [_point(mean[0], mean[1], z)
                            for _, _, mean in valid]
            means.colors = [_color(rgb, prediction.weight)
                            for _, prediction, _ in valid]
            output.markers.append(means)

            ellipses = _base_marker(
                message.header, f'prediction_{mode_name}_covariance', marker_id,
                Marker.LINE_LIST, lifetime)
            ellipses.scale.x = max(0.005, float(line_width) * 0.45)
            ellipse_predictions = [
                item for item in valid if item[0] % ellipse_stride == 0]
            if (ellipse_predictions
                    and ellipse_predictions[-1][0] != valid[-1][0]):
                ellipse_predictions.append(valid[-1])
            for _, prediction, mean in ellipse_predictions:
                if len(prediction.covariance) != 4:
                    continue
                vertices = _ellipse_segments(
                    mean, prediction.covariance, covariance_scale,
                    segment_count, z)
                ellipses.points.extend(vertices)
                ellipses.colors.extend(
                    [_color(rgb, prediction.weight) for _ in vertices])
            output.markers.append(ellipses)

            first_prediction = valid[0][1]
            label_candidates.append((
                float(first_prediction.weight), mode_name.upper(), valid[0][2]))

        if not label_candidates:
            continue
        label_candidates.sort(key=lambda candidate: candidate[0], reverse=True)
        label_position = label_candidates[0][2]
        probability_text = ' | '.join(
            f'{name} {weight * 100.0:.0f}%'
            for weight, name, _ in sorted(
                label_candidates, key=lambda candidate: candidate[1],
                reverse=True))
        label = _base_marker(
            message.header, 'prediction_labels', marker_id,
            Marker.TEXT_VIEW_FACING, lifetime)
        label.pose.position = _point(
            label_position[0], label_position[1], 0.35)
        label.scale.z = 0.15
        label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        class_name = predicted.class_name or str(predicted.class_id)
        motion_name = MOTION_NAMES.get(int(predicted.motion_state), 'UNKNOWN')
        label.text = (
            f'#{predicted.track_id} {class_name} {motion_name}\n'
            f'{probability_text}\n'
            f'{"LOST" if predicted.lifecycle_state == 2 else "TRACKED"} '
            f'obs {predicted.measurement_age:.1f}s '
            f'class {predicted.semantic_age:.1f}s')
        output.markers.append(label)

    return output


class PredictionMarkersNode(Node):
    """Bridge ``/predictions`` to an RViz-friendly marker array."""

    def __init__(self):
        super().__init__('prediction_markers_node')
        self.input_topic = self.declare_parameter(
            'predictions_topic', '/predictions').value
        self.output_topic = self.declare_parameter(
            'prediction_markers_topic', '/prediction_markers').value
        self.covariance_scale = float(self.declare_parameter(
            'visualization.covariance_scale', 2.4477).value)
        self.ellipse_segments = int(self.declare_parameter(
            'visualization.ellipse_segments', 24).value)
        self.covariance_stride = int(self.declare_parameter(
            'visualization.covariance_stride', 5).value)
        self.lifetime = float(self.declare_parameter(
            'visualization.marker_lifetime', 0.5).value)
        self.line_width = float(self.declare_parameter(
            'visualization.line_width', 0.025).value)
        self.point_diameter = float(self.declare_parameter(
            'visualization.point_diameter', 0.06).value)

        self.publisher = self.create_publisher(
            MarkerArray, self.output_topic, 10)
        self.create_subscription(
            MultiModalObjectPredictionArray,
            self.input_topic,
            self._on_predictions,
            10,
        )
        self.get_logger().info(
            f'prediction_markers_node: {self.input_topic} -> '
            f'{self.output_topic} (covariance scale={self.covariance_scale:.4f})')

    def _on_predictions(self, message):
        markers = build_prediction_markers(
            message,
            covariance_scale=self.covariance_scale,
            ellipse_segments=self.ellipse_segments,
            covariance_stride=self.covariance_stride,
            lifetime=self.lifetime,
            line_width=self.line_width,
            point_diameter=self.point_diameter,
        )
        self.publisher.publish(markers)


def main(args=None):
    """Run the prediction marker bridge."""
    rclpy.init(args=args)
    node = PredictionMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
