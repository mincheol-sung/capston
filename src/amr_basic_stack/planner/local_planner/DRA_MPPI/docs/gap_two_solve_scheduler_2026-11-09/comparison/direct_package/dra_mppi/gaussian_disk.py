"""Gaussian mass in a collision disk, without a candidate-spanning MC domain.

Isotropic normals use the noncentral chi-square CDF. General normals use
conditional-normal one-dimensional Gauss-Legendre quadrature, refined until
successive orders agree. Refinement is an error estimate, not a certified bound;
unresolved integrals fail closed. Covariance flooring, mixture weights, obstacle
independence and collision radii match the shared MC implementation.
"""
from functools import lru_cache
import time

import numpy as np
from scipy.special import ndtr, roots_legendre
from scipy.stats import ncx2

from .dynamic_risk import MonteCarloDiagnostics, MonteCarloJointCollisionProbability


@lru_cache(maxsize=8)
def _disk_nodes(order):
    nodes, weights = roots_legendre(order)
    theta = nodes * (np.pi / 2.)
    result = (np.sin(theta), np.cos(theta), weights * (np.pi / 2.))
    for a in result:
        a.setflags(write=False)
    return result


class GaussianDiskCollisionProbability(MonteCarloJointCollisionProbability):
    """Deterministic circle integration with adaptive anisotropic quadrature."""

    @property
    def sampling_mode(self):
        return 'gaussian_disk'

    def _sample_count(self, region_area):
        return 0  # No Monte Carlo samples; integration work has separate counters.

    @staticmethod
    def _integrate(mu, precision, radius, order):
        sine, cosine, weights = _disk_nodes(order)
        x = radius[:, None] * sine
        height = radius[:, None] * cosine
        a, b, c = precision[:, 0, 0], precision[:, 0, 1], precision[:, 1, 1]
        sigma_x = np.sqrt(c / (a*c-b*b))
        sigma_y = 1. / np.sqrt(c)
        z = (x-mu[:, :1]) / sigma_x[:, None]
        conditional_mean = mu[:, 1:2] - (b/c)[:, None]*(x-mu[:, :1])
        mass_y = (ndtr((height-conditional_mean)/sigma_y[:, None])
                  - ndtr((-height-conditional_mean)/sigma_y[:, None]))
        pdf_x = np.exp(-.5*z*z)/(np.sqrt(2.*np.pi)*sigma_x[:, None])
        return np.sum(pdf_x*mass_y*height*weights, axis=1)

    def evaluate(self, positions, predictions, collision_radius,
                 cache_samples=False, reuse_last_samples=False):
        points = np.asarray(positions, dtype=np.float64)
        if points.ndim != 3 or points.shape[-1] != 2 or not np.all(np.isfinite(points)):
            raise ValueError('positions must be finite with shape (K,T,2)')
        k, horizon = points.shape[:2]
        if predictions is None or predictions.empty:
            self.reset_diagnostics()
            return np.zeros((k, horizon))
        o, m, t = predictions.means.shape[:3]
        if t != horizon:
            raise ValueError('prediction and rollout horizons must match')
        radii = np.broadcast_to(np.asarray(collision_radius, dtype=float), (o,))
        if np.any(radii <= 0) or not np.all(np.isfinite(radii)):
            raise ValueError('collision radii must be finite and positive')
        start = time.perf_counter()
        inv, _ = self._gaussian_parameters(predictions)
        # Axis order K,O,M,T keeps mode and obstacle reductions explicit.
        mu = (predictions.means[None]-points[:, None, None]).reshape(-1, 2)
        precision = np.broadcast_to(inv, (k, o, m, t, 2, 2)).reshape(-1, 2, 2)
        radius = np.broadcast_to(radii[None, :, None, None], (k, o, m, t)).ravel()
        iso = ((np.abs(precision[:, 0, 1]) <= 1e-12*precision[:, 0, 0])
               & (np.abs(precision[:, 0, 0]-precision[:, 1, 1]) <= 1e-12*precision[:, 0, 0]))
        # A disk is contained in the half-space toward its center. Its 1D
        # Gaussian tail is an upper bound, including for rotated covariance.
        # Negligible tails are retained as that upper bound, never set to zero.
        distance = np.linalg.norm(mu, axis=1)
        a, b, c = precision[:, 0, 0], precision[:, 0, 1], precision[:, 1, 1]
        variance = ((c*mu[:, 0]**2-2*b*mu[:, 0]*mu[:, 1]+a*mu[:, 1]**2)
                    / (a*c-b*b) / np.maximum(distance**2, 1e-30))
        upper = np.ones(len(mu))
        outside = distance > radius
        upper[outside] = ndtr((radius[outside]-distance[outside])
                              / np.sqrt(variance[outside]))
        active = upper > 1e-10
        mass = upper.copy()
        iso &= active
        mass[iso] = ncx2.cdf(radius[iso]**2*precision[iso, 0, 0], 2,
                            np.sum(mu[iso]**2, axis=1)*precision[iso, 0, 0]) + 1e-12
        unresolved = 0
        work = 0
        error_max = 0.
        indices = np.flatnonzero(active & ~iso)
        # Chunking bounds quadrature temporaries independently of candidate count.
        for offset in range(0, len(indices), 1024):
            idx = indices[offset:offset+1024]
            low = self._integrate(mu[idx], precision[idx], radius[idx], 16)
            work += len(idx)*16
            # A minimum order based on the narrowest regularized axis prevents
            # two coarse grids from both missing a very narrow Gaussian peak.
            min_order = np.maximum(32, 4*radius[idx]*np.sqrt(np.linalg.eigvalsh(precision[idx])[:, -1]))
            for order in (32, 64, 128, 256, 512):
                high = self._integrate(mu[idx], precision[idx], radius[idx], order)
                work += len(idx)*order
                error = np.abs(high-low)
                converged = (error <= 1e-8) & (order >= min_order) & np.isfinite(high)
                mass[idx[converged]] = high[converged]+4*error[converged]+5e-8
                if np.any(converged):
                    error_max = max(error_max, float(np.max(error[converged])))
                idx = idx[~converged]
                low = high[~converged]
                min_order = min_order[~converged]
                if not len(idx):
                    break
            if len(idx):
                mass[idx] = 1.
                unresolved += len(idx)
        bad = ~np.isfinite(mass)
        unresolved += int(np.sum(bad))
        mass[bad] = 1.
        mass = np.clip(mass.reshape(k, o, m, t), 0., 1.)
        weights = predictions.mode_weights
        if weights.ndim == 2:
            weights = weights[..., None]
        marginal = np.sum(mass*weights[None], axis=2)
        joint = 1.-np.prod(1.-marginal, axis=1)
        self.last_diagnostics = MonteCarloDiagnostics(
            sampling_mode=self.sampling_mode, region_areas=np.zeros(t),
            sample_counts=np.zeros(t, dtype=np.int64), total_samples=0,
            compute_time_ms=(time.perf_counter()-start)*1000., empty_circle_count=0,
            clipped_marginal_count=0, raw_marginal_minimum=float(np.min(marginal)),
            raw_marginal_maximum=float(np.max(marginal)),
            analytic_evaluations=int(np.sum(iso)), quadrature_evaluations=work,
            unresolved_integrals=unresolved, error_estimate_max=error_max)
        return joint
