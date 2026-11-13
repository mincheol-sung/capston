#!/usr/bin/env python3
"""Small kinematic closed-loop pilot using the actual shared-cost controllers.

Not a physics/MPC/command-gate simulation, predictor calibration, or ROS timing.
Same primary K for all methods; final re-evaluation counts differ and are reported.
"""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import time

import numpy as np

from benchmark_guide_conditioned import config_from_yaml
from dra_mppi.dynamic_risk import PredictionSet
from dra_mppi.grid_costmap import DistanceField, GridMetadata
from dra_mppi.mppi_controller import MppiController
from dra_mppi.common_cost import project_path


SCENARIOS = ('open', 'offpath', 'yaw_m70', 'yaw_p90', 'yaw180', 'curve', 'corner',
             'goal_near', 'detour', 'detour_right', 'detour_large', 'narrow',
             'crossing5', 'headon', 'mixed', 'wide', 'narrow_sigma', 'multimodal20',
             'path_update', 'blocked')


def scene(name):
    grid = np.zeros((200, 200), dtype=np.int16)
    grid[:2, :] = 100
    path = np.array([[0., 0.], [2., 0.]])
    if name in ('detour', 'detour_right', 'mixed'):
        grid[90:110, 116:121] = 100
        sign = -1. if name == 'detour_right' else 1.
        path = np.array([[0., 0.], [.4, sign], [1.5, sign], [2., 0.]])
    if name == 'detour_large':
        grid[75:125, 116:122] = 100
        path = np.array([[0., 0.], [.25, 1.65], [1.65, 1.65], [2., 0.]])
    if name == 'narrow':
        grid[90, 95:145] = 100
        grid[110, 95:145] = 100
    if name == 'blocked':
        grid[:, 111:113] = 100
    if name == 'curve':
        angle = np.linspace(-np.pi/2, 0., 31)
        path = np.column_stack((1.5*np.cos(angle), 1.5+1.5*np.sin(angle)))
    if name == 'corner':
        path = np.array([[0., 0.], [1., 0.], [1., 1.5]])
    field = DistanceField.from_occupancy(grid, GridMetadata(200, 200, .05, -5., -5., 0.))
    return path, field


def predictions(now, cfg, name='crossing5'):
    times = np.arange(1, cfg.horizon_steps+1)*cfg.dt
    means = np.zeros((5, 2, cfg.horizon_steps, 2))
    base = np.array([[.9, -1.], [1.6, 1.2], [2.5, -1.3], [3., 1.8], [3.5, -.8]])
    velocity = np.array([[0., .3], [0., -.15], [-.08, .1], [-.1, -.1], [0., .05]])
    means[:, 0] = base[:, None]+velocity[:, None]*(now+times)[None, :, None]
    means[:, 1] = means[:, 0]
    means[:, 1, :, 0] += .15*times
    covariance = np.broadcast_to(np.eye(2)*.02, (5, 2, cfg.horizon_steps, 2, 2)).copy()
    if name == 'headon':
        means = np.zeros((1, 2, cfg.horizon_steps, 2))
        means[..., 0] = 2.-.25*(now+times)
        covariance = np.broadcast_to(np.eye(2)*.02, (1, 2, cfg.horizon_steps, 2, 2)).copy()
    elif name == 'wide':
        covariance *= 9.
    elif name == 'narrow_sigma':
        covariance[:] = np.eye(2)*.08**2
    elif name == 'multimodal20':
        means = np.concatenate([means+np.array([i*.3, i*.6]) for i in range(4)])
        covariance = np.tile(covariance, (4, 1, 1, 1, 1))
    return PredictionSet(means, covariance, times, np.tile([.7, .3], (len(means), 1)))


def run(name, method, seed, base, steps, stream):
    cfg = replace(base, num_rollouts=base.guide_num_rollouts, seed=seed,
                  strategy='standard' if method == 'standard' else 'guide_conditioned',
                  guide_angles_deg=(0.,) if method == 'single_anchor' else base.guide_angles_deg)
    controller = MppiController(cfg)
    path, field = scene(name)
    state, current = np.zeros(3), np.zeros(3)
    if name == 'offpath':
        state[1] = .6
    state[2] = np.deg2rad({'yaw_m70': -70, 'yaw_p90': 90, 'yaw180': 180}.get(name, 0))
    if name == 'goal_near':
        state[0], current[0] = 1.8, .2
    path_errors, heading_errors, guide_switches = [], [], 0
    previous_guide = None
    failure_reasons = {}

    success = collision = False
    minimum_clearance = float('inf')
    stop_steps = unsafe_steps = exceptions = 0
    input_change, durations = [], []
    for step in range(steps):
        pred = predictions(step*cfg.dt, cfg, name) if name in (
            'crossing5', 'headon', 'mixed', 'wide', 'narrow_sigma', 'multimodal20') else None
        if name == 'path_update' and step == 5:
            path = np.array([[0., 0.], [.7, .7], [2., .7]])

        tick = time.perf_counter()
        try:
            result = controller.compute(state, path, current, pred,
                                        visualization_rollouts=0, distance_field=field)
            control = result.control
            next_state = result.trajectory[0]
            risk = result.max_collision_probability
            unsafe_steps += int(risk > cfg.dynamic_risk.hard_threshold)
        except (RuntimeError, TimeoutError) as exc:
            control = controller._make_stopping_control(current)[0]
            from dra_mppi.mecanum_model import rollout
            next_state = rollout(state, control[None], cfg.dt)[0]
            risk = None
            exceptions += 1
            failure_reasons[str(exc)] = failure_reasons.get(str(exc), 0)+1
        if 'result' in locals() and risk is not None:
            guide_switches += int(previous_guide is not None and result.selected_mode != previous_guide)
            previous_guide = result.selected_mode
            failure_reasons[result.safety_status] = failure_reasons.get(result.safety_status, 0)+1
        closest, _, heading, _, _ = project_path(next_state[None, None, :2], path, cfg.heading_lookahead, cfg.projection_window)
        path_errors.append(float(np.linalg.norm(next_state[:2]-closest[0, 0])))
        heading_errors.append(float(abs(np.arctan2(np.sin(next_state[2]-heading[0, 0]), np.cos(next_state[2]-heading[0, 0])))))
        elapsed = (time.perf_counter()-tick)*1000.
        durations.append(elapsed)
        clearance = field.trajectory_clearances(
            next_state[None, None, :2], state[:2])[0, 0]-cfg.static_cost_robot_radius
        hit = field.trajectory_collisions(
            next_state[None, None, :2], state[:2], cfg.static_cost_robot_radius)[0]
        if pred is not None:
            # Deterministic CV truth; sample substeps for collisions between cycles.
            fractions = np.linspace(0., 1., 21)
            robot = state[:2]+fractions[:, None]*(next_state[:2]-state[:2])
            before = predictions((step-1)*cfg.dt, cfg, name).means[:, 0, 0]
            after = pred.means[:, 0, 0]
            obstacles = before[:, None]+fractions[None, :, None]*(after-before)[:, None]
            dynamic_clearance = np.linalg.norm(robot[None]-obstacles, axis=-1)-(
                cfg.dynamic_risk.robot_radius+cfg.dynamic_risk.obstacle_radius)
            clearance = min(clearance, float(dynamic_clearance.min()))
            hit |= bool(np.any(dynamic_clearance <= 0.))
        minimum_clearance = min(minimum_clearance, float(clearance))
        stop_steps += int(np.linalg.norm(control[:2]) < .03)
        input_change.append(float(np.sum(((control-current)/cfg.dt)**2)))
        stream.write(json.dumps({
            'environment': name, 'method': method, 'seed': seed, 'step': step,
            'state': state.tolist(), 'control': control.tolist(), 'risk': risk,
            'clearance_lower_bound': float(clearance), 'optimizer_ms': elapsed,
            'collision': bool(hit)})+'\n')
        state, current = next_state, control
        collision |= bool(hit)
        success = np.linalg.norm(state[:2]-path[-1]) < .15
        if success or collision:
            break
    controller.close()
    return {
        'environment': name, 'method': method, 'seed': seed, 'success': bool(success and not collision),
        'collision': collision, 'elapsed_sim_s': (step+1)*cfg.dt,
        'failure_inclusive_s': (step+1)*cfg.dt if success and not collision else steps*cfg.dt,
        'minimum_clearance_lower_bound': minimum_clearance,
        'stop_time_s': stop_steps*cfg.dt, 'risk_exceed_steps': unsafe_steps,
        'exceptions': exceptions, 'guide_switches': guide_switches,
        'failure_reasons': failure_reasons, 'mean_path_error_m': float(np.mean(path_errors)),
        'mean_heading_error_rad': float(np.mean(heading_errors)),
        'final_speed_mps': float(np.linalg.norm(current[:2])),
        'mean_input_rate_squared': float(np.mean(input_change)),
        'offline_optimizer_p99_ms': float(np.percentile(durations, 99)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seeds', type=int, default=3)
    parser.add_argument('--steps', type=int, default=100)
    parser.add_argument('--rollouts', type=int, default=100)
    parser.add_argument('--seed-start', type=int, default=101)
    parser.add_argument('--scenarios', nargs='+', choices=SCENARIOS, default=SCENARIOS)
    parser.add_argument('--methods', nargs='+', choices=('standard', 'single_anchor', 'guide'), default=['standard', 'guide'])
    parser.add_argument('--path-weight', type=float)
    parser.add_argument('--heading-weight', type=float)

    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    base = replace(config_from_yaml(), guide_num_rollouts=args.rollouts,
                   static_cost_enabled=True, guide_timeout=10.)
    if args.path_weight is not None:
        base = replace(base, tracking_path_weight=args.path_weight)
    if args.heading_weight is not None:
        base = replace(base, heading_weight=args.heading_weight)
    environments = args.scenarios
    methods = args.methods
    results = []
    with (args.output/'closed_loop_raw.jsonl').open('w') as stream:
        for name in environments:
            for seed in range(args.seed_start, args.seed_start+args.seeds):
                for method in methods:
                    results.append(run(name, method, seed, base, args.steps, stream))
    paired = []
    for name in environments:
        means = {method: np.mean([r['failure_inclusive_s'] for r in results
                                 if r['environment'] == name and r['method'] == method])
                 for method in methods}
        paired.append(means.get('guide', 0.)-means.get('standard', 0.))
    bootstrap = np.random.default_rng(73).choice(
        paired, (10000, len(environments)), replace=True).mean(axis=1)
    report = {
        'scope': 'kinematic synthetic suite; no MPC/actuators; timeout=10s for offline convergence',
        'config': asdict(base), 'primary_K': args.rollouts,
        'final_candidates': {'standard': 2, 'single_anchor': 2, 'guide': 6},
        'results': results, 'independent_environment_count': len(environments),
        'paired_environment_mean_time_difference_s': float(np.mean(paired)),
        'exploratory_environment_bootstrap_95_ci_s': np.percentile(bootstrap, [2.5, 97.5]).tolist(),
        'limitations': 'Hand-built scenes; seeds are repeats within environments. '
        'No time-budget matching, MPC dynamics, measured prediction error, perception load, '
        'or ROS deadline/physical collision relationship. Collision checks are discretized. '
        'Zero observed collisions is not a safety guarantee.',
    }
    (args.output/'closed_loop_summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({method: {
        'successes': sum(r['success'] for r in results if r['method'] == method),
        'collisions': sum(r['collision'] for r in results if r['method'] == method),
        'trials': sum(r['method'] == method for r in results)} for method in methods}, indent=2))


if __name__ == '__main__':
    main()
