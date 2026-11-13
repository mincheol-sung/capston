"""Budgeted nonlinear reach/value optimization and common group allocation.

All geometry is in the current robot frame. SLSQP produces local solutions,
not global certificates. Deadlines are cooperative at callbacks/evaluations;
the caller also rejects a late complete planning cycle.
"""
from itertools import product
from functools import lru_cache
import time
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize


class OptimizationDeadline(RuntimeError):
    """The proposal's wall-clock budget has expired."""


def reference(path, cfg, start_progress=None):
    """One common reference per snapshot, not shortened per candidate."""
    path = np.asarray(path)[:, :2]
    path = path[np.r_[True, np.linalg.norm(np.diff(path, axis=0), axis=1)>1e-9]]
    if len(path) == 1:
        return path[0].copy(), np.zeros(cfg.horizon_steps)
    seg = np.diff(path, axis=0); length = np.linalg.norm(seg, axis=1)
    arc = np.r_[0., np.cumsum(length)]
    fractions = np.clip(np.sum(-path[:-1]*seg, axis=1)/length**2, 0, 1)
    points = path[:-1]+fractions[:, None]*seg
    k = int(np.argmin(np.sum(points**2, axis=1)))
    start = (arc[k]+fractions[k]*length[k] if start_progress is None
             else np.clip(start_progress, 0., arc[-1]))
    s = np.minimum(arc[-1], start+cfg.desired_speed*cfg.dt*np.arange(1, cfg.horizon_steps+1))
    target = np.array([np.interp(s[-1], arc, path[:, axis]) for axis in range(2)])
    ids = np.clip(np.searchsorted(arc, s, side='right')-1, 0, len(seg)-1)
    heading = np.unwrap(np.arctan2(seg[ids, 1], seg[ids, 0]))
    heading -= 2*np.pi*np.round(heading[0]/(2*np.pi))
    return target, heading


@lru_cache(maxsize=16)
def _jacobian_indices(n):
    """Read-only structure shared by all solves at the same horizon length."""
    steps=np.arange(n)
    earlier=steps[None,:]<steps[:,None]
    steps.setflags(write=False);earlier.setflags(write=False)
    return steps,earlier


def rollout_jacobian(flat, n, dt):
    u = flat.reshape(n, 3)
    yaw_before = dt*np.r_[0., np.cumsum(u[:-1, 2])]
    c, s = np.cos(yaw_before), np.sin(yaw_before)
    velocity = np.column_stack((c*u[:, 0]-s*u[:, 1], s*u[:, 0]+c*u[:, 1]))
    states = np.column_stack((dt*np.cumsum(velocity, axis=0), dt*np.cumsum(u[:, 2])))
    jac = np.zeros((n, 3, n*3))
    t,earlier=_jacobian_indices(n)
    jac[t,0,3*t]=dt*c; jac[t,0,3*t+1]=-dt*s
    jac[t,1,3*t]=dt*s; jac[t,1,3*t+1]=dt*c
    jac[t,2,3*t+2]=dt
    jac[:,0,2::3]=np.where(earlier,(-dt*dt*velocity[:,1])[:,None],0.)
    jac[:,1,2::3]=np.where(earlier,(dt*dt*velocity[:,0])[:,None],0.)
    return states, np.cumsum(jac, axis=0)


class ValueProblem:
    def __init__(self, cfg, current, path, angle, depth, common_reference=None):
        self.cfg = cfg; self.n = n = cfg.horizon_steps; self.current = np.array(current)
        self.e = np.array([np.cos(angle), np.sin(angle)])
        self.perp = np.array([-np.sin(angle), np.cos(angle)])
        self.target, self.heading = reference(path, cfg) if common_reference is None else common_reference
        self.depth = depth
        g = cfg.gap
        self.weights = dict(progress=g.value_progress_weight, straight=g.value_straight_weight,
                            heading=g.value_heading_weight,
                            delta=[g.value_delta_x_weight, g.value_delta_y_weight, g.value_delta_omega_weight])
        self.target_distance = None
        dim = 3*n
        self.bounds = Bounds(np.tile(cfg.control_min, n), np.tile(cfg.control_max, n))
        rates = np.eye(dim)
        for t in range(1, n): rates[3*t:3*t+3, 3*t-3:3*t] = -np.eye(3)
        delta = np.tile(np.array(cfg.acceleration_limit)*cfg.dt, n)
        center = np.zeros(dim); center[:3] = current
        signs = np.array(list(product([-1., 1.], repeat=3)))*[1., 1., cfg.yaw_lever]
        wheel = np.kron(np.eye(n), signs)
        self.linear = LinearConstraint(np.vstack((rates, wheel)),
                np.r_[center-delta, np.full(8*n, -np.inf)],
                np.r_[center+delta, np.full(8*n, cfg.speed_limit_l1)])
        self._x = None
        self._deadline = float('inf')

    def check_deadline(self, *args):
        if time.perf_counter() >= self._deadline:
            raise OptimizationDeadline('gap optimization budget exhausted')

    def state(self, x):
        self.check_deadline()
        if self._x is None or not np.array_equal(x, self._x):
            self._x = x.copy(); self._state, self._jac = rollout_jacobian(x, self.n, self.cfg.dt)
        return self._state, self._jac

    def objective(self, x):
        state, jac = self.state(x); n = self.n; w = self.weights
        endpoint = state[-1, :2]-self.target
        lateral = state[:, :2]@self.perp
        # A fixed unwrapped heading branch avoids moving candidate references.
        heading = state[:, 2]-self.heading
        controls = x.reshape(n, 3)
        delta = np.diff(np.vstack((self.current, controls)), axis=0)
        R = np.array(w['delta'])
        parts = dict(progress=w['progress']*float(endpoint@endpoint),
                     straight=w['straight']*float(np.mean(lateral**2)),
                     heading=w['heading']*float(np.mean(heading**2)),
                     delta=float(np.mean(np.sum(delta**2*R, axis=1))))
        grad = 2*w['progress']*(endpoint@jac[-1, :2])
        grad += 2*w['straight']/n*np.einsum('t,i,tij->j', lateral, self.perp, jac[:, :2])
        grad += 2*w['heading']/n*(heading@jac[:, 2])
        du = 2*delta*R/n
        du[:-1] -= 2*delta[1:]*R/n
        return sum(parts.values()), grad+du.ravel(), parts

    def equal(self, x):
        state, jac = self.state(x)
        return np.array([state[-1, :2]@self.perp]), (self.perp@jac[-1, :2])[None]

    def inequality(self, x):
        state, jac = self.state(x)
        along = state[:, :2]@self.e; dj = np.einsum('i,tij->tj', self.e, jac[:, :2])
        # .25m nontrivial-motion requirement is symmetric for forward/reverse.
        if self.target_distance is None:
            return np.r_[along[-1]-self.cfg.gap.min_displacement_m, self.depth-along], np.vstack((dj[-1], -dj))
        # At the maximum reachable target, allow a 1cm terminal tolerance.
        lower = max(self.cfg.gap.min_displacement_m, self.target_distance-self.cfg.gap.optimization_target_tolerance_m)
        return np.r_[along[-1]-lower, self.target_distance-along[-1], self.depth-along], np.vstack((dj[-1], -dj[-1], -dj))

    def violation(self, x):
        v = self.linear.A@x
        return float(max(0., np.max(self.bounds.lb-x), np.max(x-self.bounds.ub),
                         np.max(self.linear.lb-v), np.max(v-self.linear.ub),
                         np.max(np.abs(self.equal(x)[0])), -np.min(self.inequality(x)[0])))

    def solve(self, start, deadline=float('inf')):
        tick = time.perf_counter()
        self._deadline = deadline
        self.check_deadline()
        def fg(x):
            value, gradient, _ = self.objective(x)
            return value, gradient
        constraints = [self.linear,
                       dict(type='eq', fun=lambda x:self.equal(x)[0], jac=lambda x:self.equal(x)[1]),
                       dict(type='ineq', fun=lambda x:self.inequality(x)[0], jac=lambda x:self.inequality(x)[1])]
        opt = minimize(fg, np.asarray(start).ravel(), jac=True, method='SLSQP', bounds=self.bounds,
                       constraints=constraints, callback=self.check_deadline,
                       options=dict(maxiter=self.cfg.gap.optimization_maxiter,
                                    ftol=self.cfg.gap.optimization_ftol, disp=False))
        value, _, parts = self.objective(opt.x)
        return dict(controls=opt.x.reshape(self.n, 3).copy(), trajectory=self.state(opt.x)[0].copy(),
                    cost=float(value), components=parts, success=bool(opt.success), iterations=int(opt.nit),
                    message=str(opt.message), violation=self.violation(opt.x),
                    wall_ms=(time.perf_counter()-tick)*1000)


class ReachProblem(ValueProblem):
    """Maximize endpoint distance, with NO heading/path preference in objective."""
    def objective(self, x):
        state, jac = self.state(x)
        value = -float(state[-1, :2]@self.e)
        return value, -(self.e@jac[-1, :2]), dict(negative_reach=value)


def wheel_distance_bound(cfg):
    """Obstacle-independent upper bound, not a solved reachable distance.

    ||v_world||_2 == ||v_body||_2 <= |vx|+|vy| <= wheel budget.
    Rotation is free to vary in the solve and consumes the same wheel budget.
    """
    return float(cfg.horizon_steps*cfg.dt*cfg.speed_limit_l1)


def gap_value_diagnostics():
    """Fresh per-cycle diagnostics; a fallback must never inherit an optimum."""
    return dict(wheel_distance_m=None, wheel_value_cost=None,
                wheel_value_components=None, actual_target_distance_m=None,
                actual_value_cost=None, actual_value_components=None,
                lidar_limited=None, actual_value_reused_from_wheel=False,
                wheel_trajectory_within_lidar_depth=None,
                value_difference=None, optimization_stage='not_attempted',
                wheel_reach_status='not_attempted', wheel_value_status='not_computed',
                actual_value_status='not_attempted')


def solve_gap_values(cfg, current, path, angle, depth, start,
                     common_reference=None, deadline=float('inf'), diagnostics=None):
    """Solve wheel-limited reach, then the actual LiDAR-limited endpoint value.

    Both stages allow rotation and share a deadline. The first maximizes reach;
    only the second minimizes the four-term seed cost. Legacy wheel-only value
    diagnostics remain None: no unused comparison-cost optimization is run.
    Returns the actual endpoint solution, or None on solver nonconvergence.
    These are constrained local solutions, not global-optimality certificates.
    """
    info = {} if diagnostics is None else diagnostics
    info.update(gap_value_diagnostics())
    common = reference(path, cfg) if common_reference is None else common_reference
    bound = wheel_distance_bound(cfg)
    tolerance = cfg.gap.optimization_constraint_tolerance

    def solve_stage(problem, controls, stage):
        info['optimization_stage'] = stage
        info[stage+'_status'] = 'running'
        try:
            result = problem.solve(controls, deadline)
        except OptimizationDeadline:
            info[stage+'_status'] = 'budget_exhausted'
            raise
        info[stage+'_iterations'] = result['iterations']
        info[stage+'_ms'] = result['wall_ms']
        valid = (result['success'] and result['violation'] <= tolerance
                 and np.isfinite(result['cost'])
                 and np.all(np.isfinite(result['controls'])))
        info[stage+'_status'] = 'converged' if valid else 'not_converged'
        return result if valid else None

    # No LiDAR depth, map obstacle, or path-heading objective in this solve.
    reach_problem = ReachProblem(cfg, current, path, angle, bound, common)
    reach = solve_stage(reach_problem, start, 'wheel_reach')
    if reach is None:
        return None
    distance = float(reach['trajectory'][-1, :2]@reach_problem.e)
    target = min(distance, float(depth))
    info.update(wheel_distance_m=distance, actual_target_distance_m=target,
                lidar_limited=bool(depth < distance))
    if target < cfg.gap.min_displacement_m-tolerance:
        info['optimization_stage'] = 'actual_value'
        info['actual_value_status'] = 'insufficient_distance'
        return None

    actual_problem = ValueProblem(cfg, current, path, angle, float(depth), common)
    actual_problem.target_distance = target
    actual = solve_stage(actual_problem, reach['controls'], 'actual_value')
    if actual is None:
        return None
    info.update(actual_value_cost=actual['cost'], actual_value_components=actual['components'],
                optimization_stage='complete')
    return actual


def allocate(costs, config):
    """Include nominal in the same value-based allocation; floors are reserves."""
    values = np.asarray(costs, dtype=float)
    if values.ndim != 1 or not len(values) or not np.all(np.isfinite(values)):
        raise ValueError('gap allocation requires finite nominal and guide costs')
    if len(values) > config.max_guides+1:
        raise ValueError('too many gap allocation groups')
    if len(values) == 1:
        return np.array([config.total_rollouts]), np.ones(1), config.allocation_temperature_floor
    diff = values-values.min()
    tau = max(config.allocation_temperature_floor, float(np.median(diff)))
    soft = np.exp(-diff/tau); soft /= soft.sum()
    epsilon = config.allocation_exploration
    weights = epsilon/len(values)+(1.-epsilon)*soft
    floors = np.r_[config.nominal_min_rollouts, np.full(len(values)-1,config.guide_min_rollouts)]
    remaining = config.total_rollouts-int(floors.sum())
    if remaining < 0:
        raise ValueError('gap minimum budgets exceed total rollouts')
    raw = remaining*weights; integer = np.floor(raw).astype(int)
    order = np.argsort(-(raw-integer), kind='stable')
    integer[order[:remaining-int(integer.sum())]] += 1
    counts = floors+integer
    assert counts.sum() == config.total_rollouts and np.all(counts >= 2)
    return counts, weights, tau


def score_controls(cfg, current, common_reference, direction, controls):
    """Same four terms for nominal, solved seeds, and revalidated fallbacks."""
    u = np.asarray(controls)
    yaw_before = cfg.dt*np.r_[0., np.cumsum(u[:-1, 2])]
    cs, sn = np.cos(yaw_before), np.sin(yaw_before)
    p = cfg.dt*np.cumsum(np.column_stack((cs*u[:,0]-sn*u[:,1], sn*u[:,0]+cs*u[:,1])),axis=0)
    yaw = cfg.dt*np.cumsum(u[:,2])
    target, heading = common_reference
    g = cfg.gap
    lateral = p@np.array([-direction[1],direction[0]])
    delta = np.diff(np.vstack((current,u)),axis=0)
    parts = dict(progress=float(g.value_progress_weight*np.sum((p[-1]-target)**2)),
                 straight=float(g.value_straight_weight*np.mean(lateral**2)),
                 heading=float(g.value_heading_weight*np.mean((yaw-heading)**2)),
                 delta=float(np.mean(np.sum(delta**2*np.array([
                     g.value_delta_x_weight,g.value_delta_y_weight,g.value_delta_omega_weight]),axis=1))))
    return sum(parts.values()), parts
