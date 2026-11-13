"""Robot-relative anchor ensembles using the existing Halton MPPI sampler.

Motion-primitive sampling is deliberately not part of this implementation.
Primary risk quadrature uses one local domain per guide.  A small persistent
worker pool evaluates those domains in parallel, then all final guide
representatives and Stop receive the configured final risk integration method.
Optional traversability-map costs are evaluated per rollout and again on the
final guide representatives and independent braking candidate.
"""

from concurrent.futures import ThreadPoolExecutor, wait
import copy
import time

import numpy as np

from .control_sampling import HaltonSplineSampler
from .dynamic_risk import (
    MonteCarloDiagnostics,
    make_probability_estimator,
)
from .geometry import wrap_angle
from .mecanum_model import rollout
from .mppi_controller import MppiCostTerms, MppiResult


class GuideConditionedOptimizer:
    def __init__(self, controller, *, config=None, angles=None):
        self.controller = controller
        self.cfg = cfg = controller.config if config is None else config
        self.angles = np.asarray(cfg.guide_angles_deg if angles is None else angles, dtype=float)
        if (self.angles.ndim != 1 or not self.angles.size
                or not np.all(np.isfinite(self.angles))
                or len(np.unique(self.angles % 360)) != self.angles.size):
            raise ValueError('guide angles must be finite and distinct')
        self.count = len(self.angles)
        if cfg.guide_num_rollouts < 2 * self.count:
            raise ValueError('guide_num_rollouts must reserve two samples per guide')
        if not 0.0 <= cfg.guide_anchor_blend <= 1.0:
            raise ValueError('guide_anchor_blend must be in [0,1]')
        if not np.all(np.isfinite([
                cfg.guide_anchor_blend, cfg.guide_switch_penalty,
                cfg.guide_hysteresis])) or min(
                cfg.guide_switch_penalty, cfg.guide_hysteresis) < 0:
            raise ValueError('guide penalties must be finite and nonnegative')
        std = np.asarray(cfg.guide_noise_std, dtype=float)
        if std.shape != (3,) or not np.all(np.isfinite(std)) or np.any(std < 0):
            raise ValueError('guide_noise_std must be three finite nonnegative values')
        # Independent deterministic bases; truncate to the allocated budget.
        self.noise = [HaltonSplineSampler(
            cfg.horizon_steps, knot_points=cfg.spline_knots,
            noise_std=std, scramble=cfg.halton_scramble, seed=cfg.seed + 101 + g,
        ).sample(cfg.guide_num_rollouts - 2) for g in range(self.count)]
        risk = cfg.dynamic_risk
        self.risk_workers = min(int(risk.guide_parallel_workers), self.count)
        self._risk_executor = (
            ThreadPoolExecutor(
                max_workers=self.risk_workers,
                thread_name_prefix='dra_mppi_guide',
            )
            if self.risk_workers > 1 else None
        )
        self._guide_risk_estimators = [
            make_probability_estimator(risk,
                reject_underresolved=risk.mc_reject_underresolved,
                num_samples=risk.guide_mc_samples_min,
                minimum_sigma=risk.minimum_sigma,
                seed=risk.seed + 1009 + g,
                adaptive_halton=risk.adaptive_halton_mc,
                sample_density=risk.guide_mc_sample_density,
                min_samples=risk.guide_mc_samples_min,
                max_samples=risk.guide_mc_samples_max,
            )
            for g in range(self.count)
        ]
        self._final_risk_estimator = make_probability_estimator(risk, final=True,
            reject_underresolved=risk.mc_reject_underresolved,
            num_samples=risk.final_mc_samples_min,
            minimum_sigma=risk.minimum_sigma,
            seed=risk.seed + 2003,
            adaptive_halton=risk.adaptive_halton_mc,
            sample_density=risk.final_mc_sample_density,
            min_samples=risk.final_mc_samples_min,
            max_samples=risk.final_mc_samples_max,
        )
        self.names = tuple(f'guide_{a:+g}' for a in self.angles) + ('stop',)
        self.reset()

    def reset(self):
        self.nominals = np.zeros((self.count, self.cfg.horizon_steps, 3))
        self.temperatures = np.full(self.count, self.cfg.temperature, dtype=float)
        self.initialized = np.zeros(self.count, dtype=bool)
        self.previous = None
        self.entry_yaw = None
        self.allocation_offset = 0

    def constrain(self, controls, current):
        """Joint box/wheel/rate feasibility shared with the standard strategy.

        Both endpoints lie in the convex box/wheel polytope. Interpolate toward
        each desired input just far enough to satisfy every acceleration bound.
        If measured velocity is already infeasible, wheel feasibility wins on
        the initial projection; a velocity-reference planner cannot undo it.
        """
        return self.controller._constrain_controls(controls, current)

    def anchors(self, state, path, current, goal=None):
        """Fixed world translation rays; yaw independently follows path tangent."""
        c, cfg = self.controller, self.cfg
        headings = state[2] + np.deg2rad(self.angles)
        directions = np.column_stack((np.cos(headings), np.sin(headings)))
        states = np.broadcast_to(state, (self.count, 3)).copy()
        previous = np.broadcast_to(current, (self.count, 3)).copy()
        controls = np.zeros((self.count, cfg.horizon_steps, 3))
        for t in range(cfg.horizon_steps):
            tangent = c._nearest_path_geometry(states[:, None, :], path)[3][:, 0]
            yaw_rate = np.clip(wrap_angle(tangent - states[:, 2]) / cfg.dt,
                               c._control_min[2], c._control_max[2])
            cos, sin = np.cos(states[:, 2]), np.sin(states[:, 2])
            body_direction = np.column_stack((
                cos * directions[:, 0] + sin * directions[:, 1],
                -sin * directions[:, 0] + cos * directions[:, 1]))
            axis_limit = np.where(body_direction >= 0,
                                  c._control_max[:2], -c._control_min[:2])
            axis_speed = np.divide(axis_limit, np.abs(body_direction),
                                   out=np.full_like(body_direction, np.inf),
                                   where=np.abs(body_direction) > 1e-12)
            # Translation and rotation share the wheel load. Projection below
            # handles yaw demand; the direction-speed cap handles translation.
            speed = np.minimum(cfg.desired_speed, np.minimum(
                np.min(axis_speed, axis=1),
                cfg.speed_limit_l1 / np.sum(np.abs(body_direction), axis=1)))
            if goal is not None:
                distance = np.linalg.norm(states[:, :2] - goal, axis=1)
                braking = np.sqrt(2.0 * np.min(c._acceleration_limit[:2]) * distance)
                speed = np.minimum(speed, np.minimum(braking, distance / cfg.dt))
            raw = np.column_stack((body_direction * speed[:, None], yaw_rate))
            # Per-guide current velocities, after the first step.
            for g in range(self.count):
                controls[g, t] = self.constrain(raw[g:g+1, None], previous[g])[0, 0]
                states[g] = rollout(states[g], controls[g, t:t+1], cfg.dt)[0]
            previous = controls[:, t]
        return controls

    def allocate(self, active):
        ids = np.flatnonzero(active)
        counts = np.zeros(self.count, dtype=int)
        if ids.size:
            base, remainder = divmod(self.cfg.guide_num_rollouts, len(ids))
            counts[ids] = base
            order = np.roll(ids, -(self.allocation_offset % len(ids)))
            counts[order[:remainder]] += 1
            self.allocation_offset += 1
        return counts

    def shutdown(self):
        if self._risk_executor is not None:
            self._risk_executor.shutdown(wait=True, cancel_futures=True)
            self._risk_executor = None

    @staticmethod
    def _aggregate_risk_diagnostics(diagnostics, horizon, elapsed_ms):
        """Combine per-guide work while retaining each guide/time matrix."""
        guide_count = len(diagnostics)
        areas = np.zeros((guide_count, horizon), dtype=np.float64)
        samples = np.zeros((guide_count, horizon), dtype=np.int64)
        for guide, item in enumerate(diagnostics):
            if item.region_areas.size:
                areas[guide] = item.region_areas
                samples[guide] = item.sample_counts
        finite_minima = [
            item.raw_marginal_minimum for item in diagnostics
            if np.isfinite(item.raw_marginal_minimum)
        ]
        finite_maxima = [
            item.raw_marginal_maximum for item in diagnostics
            if np.isfinite(item.raw_marginal_maximum)
        ]
        active = [item for item in diagnostics if item.sample_counts.size]
        aggregate = MonteCarloDiagnostics(
            sampling_mode=(
                f'per_guide_{active[0].sampling_mode}'
                if active else diagnostics[0].sampling_mode
            ),
            region_areas=np.sum(areas, axis=0) if active else np.empty(0),
            sample_counts=np.sum(samples, axis=0) if active else np.empty(0, dtype=np.int64),
            total_samples=int(np.sum(samples)),
            analytic_evaluations=sum(d.analytic_evaluations for d in diagnostics),
            quadrature_evaluations=sum(d.quadrature_evaluations for d in diagnostics),
            unresolved_integrals=sum(d.unresolved_integrals for d in diagnostics),
            error_estimate_max=max(d.error_estimate_max for d in diagnostics),
            candidate_sample_comparisons=sum(d.candidate_sample_comparisons for d in diagnostics),
            underresolved_steps=sum(d.underresolved_steps for d in diagnostics),
            peak_pairwise_bytes=sum(d.peak_pairwise_bytes for d in diagnostics),
            compute_time_ms=float(elapsed_ms),
            empty_circle_count=sum(item.empty_circle_count for item in diagnostics),
            clipped_marginal_count=sum(
                item.clipped_marginal_count for item in diagnostics),
            raw_marginal_minimum=(min(finite_minima) if finite_minima else 0.0),
            raw_marginal_maximum=(max(finite_maxima) if finite_maxima else 0.0),
        )
        return aggregate, areas, samples

    def _group(self, g, count, anchor, state, path, current, predictions,
               field, transform, temperature, evaluate=True):
        """One worker owns its sampler/cache and a private evaluation context."""
        c = copy.copy(self.controller)
        c.static_time_ms = 0.
        center = anchor
        if self.initialized[g]:
            center = ((1-self.cfg.guide_anchor_blend)*self.nominals[g]
                      + self.cfg.guide_anchor_blend*anchor)
        controls = np.empty((count, self.cfg.horizon_steps, 3))
        controls[0], controls[1] = center, anchor
        controls[2:] = center+self.noise[g][:count-2]
        tick = time.perf_counter()
        controls = self.constrain(controls, current)
        trajectories = rollout(state, controls, self.cfg.dt)
        sampling_ms = (time.perf_counter()-tick)*1000.
        if not evaluate:
            return controls, trajectories
        tick = time.perf_counter()
        terms, risk = c._evaluate_rollouts(
            state, path, controls, trajectories, predictions,
            distance_field=field, source_to_grid=transform,
            risk_estimator=self._guide_risk_estimators[g])
        data = c._importance_weights(terms.total, temperature)
        representative = None if data is None else np.sum(
            data[3][:, None, None]*controls, axis=0)
        return (controls, trajectories, terms, risk, data, representative,
                c.static_time_ms, sampling_ms, (time.perf_counter()-tick)*1000.)

    def _join_jobs(self, jobs, deadline):
        """Drain every job before returning, including failure paths.

        Python cannot kill running threads safely. Timeout rejects the result
        and waits for outstanding jobs; it is not a hard real-time guarantee.
        The next cycle never shares estimator state with an expired worker.
        """
        if self._risk_executor is None:
            results = []
            for job in jobs:
                results.append(job())
                if time.perf_counter() > deadline:
                    raise TimeoutError('guide worker deadline exceeded (serial)')
            return results
        futures = [self._risk_executor.submit(job) for job in jobs]
        done, pending = wait(futures, timeout=max(0., deadline-time.perf_counter()))
        if pending:
            for future in pending:
                future.cancel()
            wait(futures)
            raise TimeoutError('guide worker deadline exceeded; workers drained')
        # All tasks have finished even if an individual future raises.
        return [future.result() for future in futures]

    def compute(self, state, path, current, predictions, visual_count,
                field, transform, started, goal=None):
        if not np.all(np.isfinite(np.r_[state, current, path.ravel()])):
            raise ValueError('guide input must be finite')
        c, cfg = self.controller, self.cfg
        anchors = self.anchors(state, path, current, goal)
        anchor_paths = rollout(state, anchors, cfg.dt)

        # Keep all directional proposals; common evaluation masks individual
        # colliding samples when static_cost.enabled is true.
        active = np.ones(self.count, dtype=bool)
        counts = self.allocate(active)
        # Body-relative IDs can change world direction after a large yaw jump.
        if (self.entry_yaw is not None
                and abs(wrap_angle(state[2] - self.entry_yaw)) > np.deg2rad(30)):
            self.initialized[:] = False
            self.previous = None
        self.entry_yaw = float(state[2])
        temperatures = self.temperatures.copy()
        # Immutable cycle inputs. ROS uses a mutually exclusive executor and
        # map replacement, never in-place edits; compute has a non-reentry lock.
        for array in (state, path, current):
            array.setflags(write=False)
        ids = np.repeat(np.arange(self.count), counts)
        boundaries = np.r_[0, np.cumsum(counts)]
        slices = [slice(boundaries[g], boundaries[g+1]) for g in range(self.count)]
        stage_ms = {'anchor': (time.perf_counter()-started)*1000.}
        primary_started = time.perf_counter()
        split = cfg.guide_evaluation == 'split'
        jobs = [lambda g=g: self._group(
            g, counts[g], anchors[g], state, path, current, predictions,
            field, transform, temperatures[g], evaluate=split)
            for g in range(self.count)]
        try:
            evaluated = self._join_jobs(jobs, started+cfg.guide_timeout)
        except Exception:
            self.reset()
            raise
        controls = np.concatenate([v[0] for v in evaluated])
        trajectories = np.concatenate([v[1] for v in evaluated])
        if split:
            def merge(name):
                return np.concatenate([getattr(v[2], name) if getattr(v[2], name) is not None
                                       else np.zeros(len(v[0])) for v in evaluated])
            terms = MppiCostTerms(
                *(merge(name) for name in ('tracking', 'heading', 'speed', 'input_delta', 'risk', 'static')),
                observation=merge('observation'),
                raw={name: np.concatenate([v[2].raw[name] for v in evaluated])
                     for name in (evaluated[0][2].raw or {})},
                weighted={name: np.concatenate([v[2].weighted[name] for v in evaluated])
                          for name in (evaluated[0][2].weighted or {})})
            probabilities = np.concatenate([v[3].collision_probability for v in evaluated])
            primary_diagnostics, guide_areas, guide_samples = self._aggregate_risk_diagnostics(
                [e.last_diagnostics for e in self._guide_risk_estimators],
                cfg.horizon_steps, sum(e.last_diagnostics.compute_time_ms
                                      for e in self._guide_risk_estimators))
            if predictions is None or predictions.empty or cfg.dynamic_risk.mode == 'off':
                radius = cfg.dynamic_risk.robot_radius+cfg.dynamic_risk.obstacle_radius
                for g, group in enumerate(slices):
                    centers = trajectories[group, :, :2]
                    guide_areas[g] = np.prod(np.ptp(centers, axis=0)+2*radius, axis=1)
                    guide_samples[g] = [self._guide_risk_estimators[g]._sample_count(a)
                                        for a in guide_areas[g]]
            c.static_time_ms += sum(v[6] for v in evaluated)
            stage_ms['worker_sampling_sum'] = sum(v[7] for v in evaluated)
            stage_ms['worker_evaluation_sum'] = sum(v[8] for v in evaluated)
        else:
            terms, risk = c._evaluate_rollouts(
                state, path, controls, trajectories, predictions,
                distance_field=field, source_to_grid=transform)
            probabilities = risk.collision_probability
            primary_diagnostics = c._risk_estimator.last_diagnostics
            guide_areas = np.zeros((self.count, cfg.horizon_steps))
            guide_samples = np.zeros((self.count, cfg.horizon_steps), dtype=int)
        c._risk_estimator.last_diagnostics = primary_diagnostics
        stage_ms['primary_wall'] = (time.perf_counter()-primary_started)*1000.
        costs = terms.total
        weights = np.zeros(len(controls))
        representatives = anchors.copy()
        valid = np.zeros(self.count, dtype=bool)
        ess, normalizers = np.zeros(self.count), np.zeros(self.count)
        reasons = [''] * self.count
        for g, group in enumerate(slices):
            data = evaluated[g][4] if split else c._importance_weights(costs[group], temperatures[g])
            if data is None:
                reasons[g] = 'all_samples_invalid'
                self.initialized[g] = False
                continue
            w, normalizer = data[3:]
            weights[group] = w
            representatives[g] = (evaluated[g][5] if split else
                                   np.sum(w[:, None, None]*controls[group], axis=0))
            valid[g] = True
            ess[g] = 1.0/np.sum(w*w)
            normalizers[g] = normalizer
            self.temperatures[g] = c._adapt_temperature(temperatures[g], normalizer)
        representatives = self.constrain(representatives, current)
        # Evaluate valid representatives and a separate acceleration-limited
        # stopping trajectory derived from the measured current velocity.
        final_ids = np.r_[np.flatnonzero(valid), self.count]
        stopping_control = c._make_stopping_control(current)
        all_controls = np.concatenate((representatives,
                                       stopping_control[None, ...]))
        all_paths = rollout(state, all_controls, cfg.dt)
        final_started = time.perf_counter()
        final_terms, final_risk = c._evaluate_rollouts(
            state, path, all_controls[final_ids], all_paths[final_ids], predictions,
            distance_field=field, source_to_grid=transform,
            risk_estimator=self._final_risk_estimator)
        if time.perf_counter()-started > cfg.guide_timeout:
            self.reset()
            raise TimeoutError('guide cycle deadline exceeded after representative recheck')
        final_diagnostics = self._final_risk_estimator.last_diagnostics
        final_static = np.zeros(self.count + 1, dtype=bool)
        if final_terms.static is not None:
            final_static[final_ids] = np.isinf(final_terms.static)
        c.representative_time_ms = (time.perf_counter()-final_started)*1000.
        final_costs = np.full(self.count + 1, np.inf)
        final_costs[final_ids] = final_terms.total
        final_static_costs = np.full(self.count + 1, np.inf)
        final_static_costs[final_ids] = final_terms.static
        final_cp = np.zeros((self.count + 1, cfg.horizon_steps))
        final_cp[final_ids] = final_risk.collision_probability
        safe = (np.isfinite(final_costs)
                & (np.max(final_cp, axis=1) <= cfg.dynamic_risk.hard_threshold)
                & ~final_static)
        selection = final_costs.copy()
        selection[~safe] = np.inf
        if self.previous is not None and self.previous < self.count:
            # Penalize only transitions between moving guides. Stop is a
            # fallback, not a candidate that competes with safe motion.
            penalties = np.full(
                self.count + 1,
                cfg.guide_switch_penalty,
                dtype=float,
            )
            penalties[self.previous] = 0.0
            penalties[self.count] = 0.0
            selection += penalties
        status = 'ok'
        moving_safe = safe[:self.count]
        if not np.any(moving_safe):
            selected = self.count
            if not safe[self.count]:
                # Hold is not claimed collision-free.
                status = 'no_safe_candidate_hold'
        else:
            # A safe moving guide always takes precedence over Stop, regardless
            # of their relative costs. Compare costs only among moving guides.
            selection[self.count] = np.inf
            selected = int(np.argmin(selection[:self.count]))
            if (self.previous is not None
                    and self.previous < self.count
                    and moving_safe[self.previous]
                    and selected != self.previous
                    and selection[selected] + cfg.guide_hysteresis
                    >= final_costs[self.previous]):
                selected = self.previous
        for g in range(self.count):
            if valid[g]:
                self.nominals[g, :-1] = representatives[g, 1:]
                self.nominals[g, -1] = representatives[g, -1]
                self.initialized[g] = True
            else:
                self.initialized[g] = False
            if valid[g] and not safe[g]:
                reasons[g] = (
                    'representative_static_collision'
                    if final_static[g]
                    else 'representative_dynamic_risk'
                )
        # Retain Stop for diagnostics. It does not delay restarting once any
        # moving guide becomes safe again.
        self.previous = selected
        # Equal representation in RViz, keeping group-local weights visible.
        groups = [g for g in range(self.count) if counts[g]]
        indices = []
        if groups:
            quota, rem = divmod(max(0, int(visual_count)), len(groups))
            for i, g in enumerate(groups):
                group = slices[g]
                n = min(counts[g], quota + (i < rem))
                indices.extend((group.start + np.argsort(-weights[group])[:n]).tolist())
        indices = np.asarray(indices, dtype=int)
        is_stop = selected == self.count
        finite = costs[np.isfinite(costs)]
        selected_weights = np.array([1.0]) if is_stop else weights[slices[selected]]
        return MppiResult(
            control=all_controls[selected, 0].copy(),
            control_sequence=all_controls[selected].copy(),
            reference_state=all_paths[selected, 0].copy(),
            reference_sequence=np.concatenate((all_paths[selected], all_controls[selected]), axis=1),
            trajectory=all_paths[selected], sampled_controls=controls,
            sampled_rollouts=trajectories[indices], rollout_costs=costs,
            weights=weights, cost_terms=terms, collision_probabilities=probabilities,
            cost=float(final_costs[selected]),
            max_collision_probability=float(np.max(final_cp[selected])),
            effective_sample_size=1.0 if is_stop else float(ess[selected]),
            temperature=0.0 if is_stop else float(temperatures[selected]),
            next_temperature=0.0 if is_stop else float(self.temperatures[selected]),
            weight_normalizer=1.0 if is_stop else float(normalizers[selected]),
            minimum_cost=float(np.min(finite)) if len(finite) else float('inf'),
            maximum_cost=float(np.max(finite)) if len(finite) else float('inf'),
            minimum_weight=float(np.min(selected_weights)),
            maximum_weight=float(np.max(selected_weights)),
            compute_time_ms=(time.perf_counter() - started) * 1000,
            strategy='guide_conditioned', selected_mode=self.names[selected],
            control_mode_ids=ids, mode_control_sequences=all_controls,
            mode_trajectories=all_paths, mode_final_costs=final_costs,
            mode_selection_costs=selection, mode_collision_probabilities=final_cp,
            mode_temperatures=np.r_[temperatures, 0.0],
            mode_next_temperatures=np.r_[self.temperatures, 0.0],
            mode_effective_sample_sizes=np.r_[ess, 1.0],
            mode_static_costs=final_static_costs,
            sampled_rollout_indices=indices,
            primary_risk_time_ms=primary_diagnostics.compute_time_ms,
            final_risk_diagnostics=final_diagnostics,
            final_risk_time_ms=final_diagnostics.compute_time_ms,
            candidate_names=self.names, guide_anchors=anchor_paths,
            guide_active_mask=active, guide_rollout_counts=counts,
            guide_rejection_reasons=tuple(reasons),
            final_static_collisions=final_static, safety_status=status,
            selected_static_cost=float(final_static_costs[selected]),
            guide_mc_region_areas=guide_areas,
            guide_mc_sample_counts=guide_samples,
            final_mc_region_areas=final_diagnostics.region_areas.copy(),
            final_mc_sample_counts=final_diagnostics.sample_counts.copy(),
            selected_cost_raw={name: float(value[np.flatnonzero(final_ids == selected)[0]])
                               for name, value in (final_terms.raw or {}).items()},
            selected_cost_weighted={name: float(value[np.flatnonzero(final_ids == selected)[0]])
                                    for name, value in (final_terms.weighted or {}).items()},
            stage_ms=stage_ms,

        )
