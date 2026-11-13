"""Low-discrepancy, temporally smooth control sampling for MPPI."""

import numpy as np
from scipy.interpolate import make_interp_spline
from scipy.special import ndtri
from scipy.stats import qmc


class HaltonSplineSampler:
    """Generate Gaussian Halton control perturbations through B-splines."""

    def __init__(
        self,
        horizon_steps,
        control_dimension=3,
        knot_points=4,
        noise_std=(1.0, 1.0, 1.0),
        seed=0,
        scramble=True,
    ):
        self.horizon_steps = int(horizon_steps)
        self.control_dimension = int(control_dimension)
        self.knot_points = int(knot_points)
        self.noise_std = np.asarray(noise_std, dtype=np.float64)
        if self.horizon_steps < 1:
            raise ValueError('horizon_steps must be positive')
        if self.control_dimension < 1:
            raise ValueError('control_dimension must be positive')
        if self.knot_points < 2:
            raise ValueError('knot_points must be at least 2')
        if self.noise_std.shape != (self.control_dimension,):
            raise ValueError('noise_std must contain one value per control axis')
        if np.any(self.noise_std < 0.0):
            raise ValueError('noise_std values must be non-negative')

        self._engine = qmc.Halton(
            d=self.knot_points * self.control_dimension,
            scramble=bool(scramble),
            seed=int(seed),
        )

    def sample(self, num_samples):
        """Return perturbations shaped ``(num_samples, horizon, control)``."""
        count = int(num_samples)
        if count < 0:
            raise ValueError('num_samples must not be negative')
        if count == 0:
            return np.empty(
                (0, self.horizon_steps, self.control_dimension),
                dtype=np.float64,
            )

        uniform = self._engine.random(count)
        # ndtri(0) and ndtri(1) are infinite. Scrambling normally avoids the
        # endpoints, but clipping also covers deterministic/non-scrambled use.
        gaussian = ndtri(np.clip(uniform, 1e-9, 1.0 - 1e-9))
        knots = gaussian.reshape(
            count,
            self.knot_points,
            self.control_dimension,
        )

        knot_time = np.linspace(0.0, 1.0, self.knot_points)
        horizon_time = np.linspace(0.0, 1.0, self.horizon_steps)
        degree = min(3, self.knot_points - 1)
        spline = make_interp_spline(knot_time, knots, k=degree, axis=1)
        samples = spline(horizon_time)
        return np.asarray(samples, dtype=np.float64) * self.noise_std
