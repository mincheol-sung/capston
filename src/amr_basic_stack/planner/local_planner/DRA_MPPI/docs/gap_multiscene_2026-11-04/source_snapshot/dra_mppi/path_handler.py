"""Reference-path cropping and terminal-goal handling."""

import math

import numpy as np


def select_local_path(path, current_xy, lookahead_distance=1.0, max_points=80):
    """Return a forward path window ending at least lookahead_distance away."""
    points = np.asarray(path, dtype=np.float64)
    current = np.asarray(current_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0 or points.shape[1] < 2:
        raise ValueError('path must contain x/y points')

    distances = np.linalg.norm(points[:, :2] - current[None, :2], axis=1)
    start = int(np.argmin(distances))
    remaining = points[start:, :2]
    if remaining.shape[0] == 1:
        return remaining

    segment_lengths = np.linalg.norm(np.diff(remaining, axis=0), axis=1)
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    end = int(np.searchsorted(cumulative, lookahead_distance, side='left')) + 1
    end = max(2, min(end, remaining.shape[0], int(max_points)))
    return remaining[:end]


def goal_reached(path, current_xy, tolerance):
    """Return whether current_xy is within tolerance of the global goal."""
    points = np.asarray(path, dtype=np.float64)
    if points.ndim != 2 or points.shape[0] == 0:
        return False
    return float(np.linalg.norm(points[-1, :2] - current_xy[:2])) <= tolerance


def heading_error(goal_yaw, current_yaw):
    """Return the signed shortest rotation from current_yaw to goal_yaw."""
    return float(np.arctan2(
        np.sin(float(goal_yaw) - float(current_yaw)),
        np.cos(float(goal_yaw) - float(current_yaw)),
    ))


def build_heading_alignment_reference(
    current_state,
    goal_yaw,
    dt,
    horizon_steps,
    max_yaw_rate,
    yaw_acceleration_limit,
    gain,
    initial_yaw_rate=0.0,
):
    """Build an in-place, acceleration-limited terminal yaw trajectory."""
    if dt <= 0.0 or horizon_steps < 1:
        raise ValueError('dt and horizon_steps must be positive')
    if max_yaw_rate <= 0.0 or yaw_acceleration_limit <= 0.0 or gain <= 0.0:
        raise ValueError('goal-heading rate, acceleration, and gain must be positive')

    state = np.asarray(current_state, dtype=np.float64)
    if state.shape != (3,):
        raise ValueError('current_state must be [x, y, yaw]')

    sequence = np.zeros((int(horizon_steps), 6), dtype=np.float64)
    sequence[:, :2] = state[:2]
    yaw = float(state[2])
    yaw_rate = float(np.clip(initial_yaw_rate, -max_yaw_rate, max_yaw_rate))
    max_rate_step = float(yaw_acceleration_limit * dt)

    for step in range(int(horizon_steps)):
        error = heading_error(goal_yaw, yaw)
        if abs(error) <= 1e-9:
            yaw = float(goal_yaw)
            yaw_rate = 0.0
        else:
            # The square-root bound gives enough distance to decelerate before
            # the target; proportional limiting makes the final approach calm.
            stopping_rate = math.sqrt(2.0 * yaw_acceleration_limit * abs(error))
            desired_rate = math.copysign(
                min(max_yaw_rate, gain * abs(error), stopping_rate),
                error,
            )
            yaw_rate += float(np.clip(
                desired_rate - yaw_rate,
                -max_rate_step,
                max_rate_step,
            ))
            yaw_step = yaw_rate * dt
            if yaw_step * error > 0.0 and abs(yaw_step) >= abs(error):
                yaw = float(goal_yaw)
                yaw_rate = 0.0
            else:
                yaw += yaw_step

        sequence[step, 2] = float(np.arctan2(np.sin(yaw), np.cos(yaw)))
        sequence[step, 5] = yaw_rate

    return sequence


def build_terminal_pose_reference(
    current_state,
    goal_state,
    current_control,
    dt,
    horizon_steps,
    max_linear_speed,
    linear_gain,
    max_yaw_rate,
    yaw_gain,
    acceleration_limit,
):
    """Build a slow acceleration-limited reference to the exact goal pose.

    Translation is expressed in the robot body frame, while the simulated
    pose remains in the planning frame. The square-root speed bounds leave
    enough distance to brake, and proportional bounds calm the final
    approach.
    """
    if dt <= 0.0 or horizon_steps < 1:
        raise ValueError('dt and horizon_steps must be positive')
    if max_linear_speed <= 0.0 or linear_gain <= 0.0:
        raise ValueError('terminal linear speed and gain must be positive')
    if max_yaw_rate <= 0.0 or yaw_gain <= 0.0:
        raise ValueError('terminal yaw rate and gain must be positive')

    state = np.asarray(current_state, dtype=np.float64).copy()
    goal = np.asarray(goal_state, dtype=np.float64)
    previous = np.asarray(current_control, dtype=np.float64).copy()
    acceleration = np.asarray(acceleration_limit, dtype=np.float64)
    if state.shape != (3,) or goal.shape != (3,) or previous.shape != (3,):
        raise ValueError('state, goal and control must contain three values')
    if acceleration.shape != (3,) or np.any(acceleration <= 0.0):
        raise ValueError(
            'acceleration_limit must contain three positive values'
        )
    if not np.all(np.isfinite(np.r_[state, goal, previous, acceleration])):
        raise ValueError('terminal reference inputs must be finite')

    sequence = np.zeros((int(horizon_steps), 6), dtype=np.float64)
    max_delta = acceleration * float(dt)
    translational_acceleration = float(np.min(acceleration[:2]))

    for step in range(int(horizon_steps)):
        world_error = goal[:2] - state[:2]
        distance = float(np.linalg.norm(world_error))
        if distance <= 1e-12:
            desired_world_velocity = np.zeros(2, dtype=np.float64)
        else:
            braking_speed = math.sqrt(
                2.0 * translational_acceleration * distance
            )
            speed = min(
                float(max_linear_speed),
                float(linear_gain) * distance,
                braking_speed,
                distance / float(dt),
            )
            desired_world_velocity = world_error * (speed / distance)

        cosine, sine = math.cos(state[2]), math.sin(state[2])
        desired_body_velocity = np.array((
            cosine * desired_world_velocity[0]
            + sine * desired_world_velocity[1],
            -sine * desired_world_velocity[0]
            + cosine * desired_world_velocity[1],
        ))

        yaw_error = heading_error(goal[2], state[2])
        if abs(yaw_error) <= 1e-12:
            desired_yaw_rate = 0.0
        else:
            stopping_rate = math.sqrt(
                2.0 * acceleration[2] * abs(yaw_error)
            )
            desired_yaw_rate = math.copysign(
                min(
                    float(max_yaw_rate),
                    float(yaw_gain) * abs(yaw_error),
                    stopping_rate,
                ),
                yaw_error,
            )

        desired = np.array((
            desired_body_velocity[0],
            desired_body_velocity[1],
            desired_yaw_rate,
        ))
        control = previous + np.clip(desired - previous, -max_delta, max_delta)
        control[2] = np.clip(control[2], -max_yaw_rate, max_yaw_rate)

        cosine, sine = math.cos(state[2]), math.sin(state[2])
        world_velocity = np.array((
            cosine * control[0] - sine * control[1],
            sine * control[0] + cosine * control[1],
        ))
        state[:2] += world_velocity * dt
        yaw_step = control[2] * dt
        if yaw_step * yaw_error > 0.0 and abs(yaw_step) >= abs(yaw_error):
            state[2] = float(goal[2])
            control[2] = 0.0
        else:
            state[2] = float(np.arctan2(
                np.sin(state[2] + yaw_step),
                np.cos(state[2] + yaw_step),
            ))

        sequence[step, :3] = state
        sequence[step, 3:] = control
        previous = control

    return sequence


class GoalCompletionLatch:
    """Remember one goal arrival until an explicit new goal resets it."""

    def __init__(self):
        self.completed = False
        self.hold_state = None

    def reset(self):
        self.completed = False
        self.hold_state = None

    def complete_if_reached(
        self,
        path,
        current_state,
        position_tolerance,
        goal_yaw=None,
        yaw_tolerance=None,
        hold_state=None,
    ):
        """Latch only after both position and optional terminal yaw converge."""
        if self.completed or not goal_reached(
            path,
            current_state,
            position_tolerance,
        ):
            return False
        if goal_yaw is not None:
            if yaw_tolerance is None or yaw_tolerance <= 0.0:
                raise ValueError('a positive yaw_tolerance is required')
            if abs(heading_error(goal_yaw, current_state[2])) > yaw_tolerance:
                return False
        target = current_state if hold_state is None else hold_state
        self.hold_state = np.asarray(target, dtype=np.float64).copy()
        if (
            self.hold_state.shape != (3,)
            or not np.all(np.isfinite(self.hold_state))
        ):
            raise ValueError('hold_state must contain three finite values')
        self.completed = True
        return True
