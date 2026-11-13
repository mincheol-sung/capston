"""Turn tracker IMM posteriors into multi-modal future Gaussians."""
from amr_interfaces.msg import (
    GaussianPrediction2D,
    ImmModeState,
    ImmTrackedObjectArray,
    MotionModePrediction,
    MultiModalObjectPrediction,
    MultiModalObjectPredictionArray,
)
from motion_predictor.core.predictor import ImmMotionPredictor
import numpy as np
from object_tracker.core.config import TrackerConfig
from object_tracker.core.imm import MODE_COUNT
import rclpy
from rclpy.node import Node


MODEL_PARAMETERS = (
    'sigma_a',
    'sigma_a_ct',
    'sigma_omega_cv',
    'sigma_omega_ct',
    'sigma_omega0',
    'cv_omega_decay_time',
    'minimum_mode_probability',
    'sigma_position_static',
    'static_velocity_decay_time',
)


class MotionPredictorNode(Node):
    """Convert current IMM mode posteriors to finite-horizon predictions."""

    def __init__(self):
        super().__init__('motion_predictor_node')
        self.input_topic = self.declare_parameter(
            'tracked_modes_topic', '/tracked_object_modes').value
        self.output_topic = self.declare_parameter(
            'predictions_topic', '/predictions').value
        prediction_dt = float(
            self.declare_parameter('prediction_dt', 0.2).value)
        horizon = float(
            self.declare_parameter('prediction_horizon', 4.5).value)
        propagate = bool(self.declare_parameter(
            'propagate_mode_probabilities', False).value)

        model_config = TrackerConfig()
        defaults = TrackerConfig.field_defaults()
        for name in MODEL_PARAMETERS:
            default = defaults[name]
            value = self.declare_parameter(name, default).value
            setattr(model_config, name, type(default)(value))
        model_config.validate()
        self.predictor = ImmMotionPredictor(
            model_config,
            prediction_dt=prediction_dt,
            horizon=horizon,
            propagate_mode_probabilities=propagate,
        )

        self.create_subscription(
            ImmTrackedObjectArray, self.input_topic, self._on_tracks, 10)
        self.pub = self.create_publisher(
            MultiModalObjectPredictionArray, self.output_topic, 10)
        probability_mode = (
            'TPM propagation' if propagate else 'fixed posterior weights')
        topic_description = (
            f'motion_predictor_node: {self.input_topic} -> '
            f'{self.output_topic}, ')
        description = topic_description + (
            f'dt={prediction_dt:.3f}s horizon={horizon:.3f}s, '
            f'{probability_mode}')
        self.get_logger().info(description)

    def _on_tracks(self, message):
        output = MultiModalObjectPredictionArray()
        output.header = message.header
        for tracked in message.objects:
            try:
                states, covariances, probabilities = self._components(tracked)
                times, means, covariances_xy, weights = self.predictor.predict(
                    states,
                    covariances,
                    probabilities,
                    tracked.mean_sojourn_cv,
                    tracked.mean_sojourn_ct,
                    tracked.mean_sojourn_static,
                    tracked.static_to_cv,
                    tracked.cv_to_static,
                    tracked.ct_to_static,
                )
            except (ValueError, IndexError) as exc:
                self.get_logger().warning(
                    f'Skipping malformed IMM track #{tracked.track_id}: {exc}',
                    throttle_duration_sec=2.0,
                )
                continue

            prediction = MultiModalObjectPrediction()
            prediction.track_id = tracked.track_id
            prediction.class_id = tracked.class_id
            prediction.class_name = tracked.class_name
            prediction.radius = tracked.radius
            prediction.motion_state = tracked.motion_state
            prediction.lifecycle_state = tracked.lifecycle_state
            prediction.measurement_age = tracked.measurement_age
            prediction.semantic_age = tracked.semantic_age
            prediction.last_measurement_source = tracked.last_measurement_source
            prediction.class_confidence = tracked.confidence
            # Include the posterior epoch explicitly. DRA uses this t=0 sample
            # as the fixed safety-disc centre for STATIC/UNKNOWN tracks.
            prediction.time_offsets = [0.0] + [float(value) for value in times]
            current_weights = probabilities / np.sum(probabilities)
            for mode_index in range(MODE_COUNT):
                mode = MotionModePrediction()
                mode.model_type = mode_index
                current = GaussianPrediction2D()
                current.weight = float(current_weights[mode_index])
                current.mean_x = float(states[mode_index, 0])
                current.mean_y = float(states[mode_index, 1])
                current.covariance = [
                    float(value)
                    for value in covariances[mode_index, :2, :2].flatten()
                ]
                mode.points.append(current)
                for step in range(times.size):
                    point = GaussianPrediction2D()
                    point.weight = float(weights[mode_index, step])
                    point.mean_x = float(means[mode_index, step, 0])
                    point.mean_y = float(means[mode_index, step, 1])
                    point.covariance = [
                        float(value)
                        for value in covariances_xy[mode_index, step].flatten()
                    ]
                    mode.points.append(point)
                prediction.modes.append(mode)
            output.objects.append(prediction)
        self.pub.publish(output)

    @staticmethod
    def _components(tracked):
        """Deserialize CV/CT/STATIC components in canonical mode order."""
        states = np.empty((MODE_COUNT, 5), dtype=float)
        covariances = np.empty((MODE_COUNT, 5, 5), dtype=float)
        probabilities = np.empty(MODE_COUNT, dtype=float)
        found = set()
        for mode in tracked.modes:
            index = int(mode.model_type)
            if index not in range(MODE_COUNT):
                continue
            if index in found:
                raise ValueError(f'duplicate mode {index}')
            states[index] = np.asarray(mode.state, dtype=float)
            covariances[index] = np.asarray(
                mode.covariance, dtype=float).reshape(5, 5)
            probabilities[index] = float(mode.probability)
            found.add(index)
        if found != set(range(MODE_COUNT)):
            raise ValueError('CV, CT and STATIC modes are required; replay old bags from detections')
        return states, covariances, probabilities


def main(args=None):
    """Run the ROS 2 motion-predictor node."""
    rclpy.init(args=args)
    node = MotionPredictorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
