import numpy as np

from dra_mppi.control_sampling import HaltonSplineSampler


def test_halton_spline_sampler_is_reproducible_and_has_expected_shape():
    first = HaltonSplineSampler(
        horizon_steps=20,
        control_dimension=3,
        knot_points=5,
        noise_std=(0.1, 0.2, 0.3),
        seed=9,
    ).sample(32)
    second = HaltonSplineSampler(
        horizon_steps=20,
        control_dimension=3,
        knot_points=5,
        noise_std=(0.1, 0.2, 0.3),
        seed=9,
    ).sample(32)

    assert first.shape == (32, 20, 3)
    assert np.all(np.isfinite(first))
    assert np.allclose(first, second)


def test_spline_sampling_is_temporally_continuous():
    samples = HaltonSplineSampler(
        horizon_steps=30,
        control_dimension=3,
        knot_points=4,
        seed=2,
    ).sample(64)
    first_difference = np.diff(samples, axis=1)
    second_difference = np.diff(samples, n=2, axis=1)

    assert np.max(np.abs(first_difference)) < 4.0
    assert np.all(np.isfinite(second_difference))
