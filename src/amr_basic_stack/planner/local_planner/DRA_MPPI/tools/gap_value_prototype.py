"""Offline research prototype: obstacle-free value optimization per LiDAR gap.

Uses the actual nonlinear mecanum rollout, analytic derivatives, and SLSQP.
No production planner changes. Report feasible local solutions, never global
optima. A central gap direction constrains the endpoint; intermediate motion
may bend and must pass a separate swept-body collision check.
"""
from itertools import product
import time
import numpy as np
from scipy.optimize import Bounds, LinearConstraint, minimize


WEIGHTS = dict(progress=8., straight=4., heading=3., delta=[.5, .5, .055])


def reference(path, cfg):
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
    start = arc[k]+fractions[k]*length[k]
    s = np.minimum(arc[-1], start+cfg.desired_speed*cfg.dt*np.arange(1, cfg.horizon_steps+1))
    target = np.array([np.interp(s[-1], arc, path[:, axis]) for axis in range(2)])
    ids = np.clip(np.searchsorted(arc, s, side='right')-1, 0, len(seg)-1)
    heading = np.unwrap(np.arctan2(seg[ids, 1], seg[ids, 0]))
    heading -= 2*np.pi*np.round(heading[0]/(2*np.pi))
    return target, heading


def rollout_jacobian(flat, n, dt):
    u = flat.reshape(n, 3)
    yaw_before = dt*np.r_[0., np.cumsum(u[:-1, 2])]
    c, s = np.cos(yaw_before), np.sin(yaw_before)
    velocity = np.column_stack((c*u[:, 0]-s*u[:, 1], s*u[:, 0]+c*u[:, 1]))
    states = np.column_stack((dt*np.cumsum(velocity, axis=0), dt*np.cumsum(u[:, 2])))
    jac = np.zeros((n, 3, n*3))
    for t in range(n):
        jac[t, 0, 3*t] = dt*c[t]; jac[t, 0, 3*t+1] = -dt*s[t]
        jac[t, 1, 3*t] = dt*s[t]; jac[t, 1, 3*t+1] = dt*c[t]
        jac[t, 2, 3*t+2] = dt
        jac[t, 0, 2:3*t:3] = -dt*dt*velocity[t, 1]
        jac[t, 1, 2:3*t:3] = dt*dt*velocity[t, 0]
    return states, np.cumsum(jac, axis=0)


class ValueProblem:
    def __init__(self, cfg, current, path, angle, depth):
        self.cfg = cfg; self.n = n = cfg.horizon_steps; self.current = np.array(current)
        self.e = np.array([np.cos(angle), np.sin(angle)])
        self.perp = np.array([-np.sin(angle), np.cos(angle)])
        self.target, self.heading = reference(path, cfg)
        self.depth = depth; self.weights = WEIGHTS
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

    def state(self, x):
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
            return np.r_[along[-1]-.25, self.depth-along], np.vstack((dj[-1], -dj))
        # At the maximum reachable target, allow a 1cm terminal tolerance.
        lower = max(.25, self.target_distance-.01)
        return np.r_[along[-1]-lower, self.target_distance-along[-1], self.depth-along], np.vstack((dj[-1], -dj[-1], -dj))

    def violation(self, x):
        v = self.linear.A@x
        return float(max(0., np.max(self.bounds.lb-x), np.max(x-self.bounds.ub),
                         np.max(self.linear.lb-v), np.max(v-self.linear.ub),
                         np.max(np.abs(self.equal(x)[0])), -np.min(self.inequality(x)[0])))

    def solve(self, start, maxiter=45):
        tick = time.perf_counter()
        def fg(x):
            value, gradient, _ = self.objective(x)
            return value, gradient
        constraints = [self.linear,
                       dict(type='eq', fun=lambda x:self.equal(x)[0], jac=lambda x:self.equal(x)[1]),
                       dict(type='ineq', fun=lambda x:self.inequality(x)[0], jac=lambda x:self.inequality(x)[1])]
        opt = minimize(fg, np.asarray(start).ravel(), jac=True, method='SLSQP', bounds=self.bounds,
                       constraints=constraints, options=dict(maxiter=maxiter, ftol=1e-6, disp=False))
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


def allocate(costs, total=500, nominal=200, minimum=30):
    """Reserve nominal, floor per gap, softmax remainder, exact integer sum."""
    if not len(costs): return np.array([total]), np.empty(0), 1.
    values = np.asarray(costs); diff = values-values.min()
    tau = max(1., float(np.median(diff)))
    soft = np.exp(-diff/tau); soft /= soft.sum()
    weights = .20/len(values)+.80*soft
    remaining = total-nominal-minimum*len(values)
    raw = remaining*weights; integer = np.floor(raw).astype(int)
    order = np.argsort(-(raw-integer), kind='stable')
    integer[order[:remaining-int(integer.sum())]] += 1
    counts = np.r_[nominal, minimum+integer]
    assert counts.sum() == total and np.all(counts >= 2)
    return counts, weights, tau


def check_derivatives(cfg):
    """Independent central differences for nonlinear rollout/cost constraints."""
    rng = np.random.default_rng(23)
    problem = ValueProblem(cfg, np.zeros(3), np.array([[0., 0.], [1., .3], [2., .6]]), .7, 1.)
    x = rng.normal(0, .2, 3*cfg.horizon_steps); eps = 1e-6
    checks = []
    for fun, analytic in [(lambda z:problem.objective(z)[0], problem.objective(x)[1]),
                          (lambda z:problem.equal(z)[0], problem.equal(x)[1]),
                          (lambda z:problem.inequality(z)[0], problem.inequality(x)[1])]:
        numeric = np.stack([(np.asarray(fun(x+eps*e))-np.asarray(fun(x-eps*e)))/(2*eps)
                            for e in np.eye(len(x))], axis=-1)
        error = float(np.max(np.abs(numeric-analytic)))
        np.testing.assert_allclose(numeric, analytic, atol=2e-7, rtol=2e-6)
        checks.append(error)
    return checks
