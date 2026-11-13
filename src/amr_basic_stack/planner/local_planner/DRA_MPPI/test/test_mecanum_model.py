import numpy as np

from dra_mppi.mecanum_model import rollout


def test_rollout_supports_strafe_and_yaw():
    controls = np.array([
        [1.0, 0.0, np.pi / 2.0],
        [0.0, 1.0, 0.0],
    ])
    states = rollout(np.zeros(3), controls, dt=1.0)

    assert np.allclose(states[0], [1.0, 0.0, np.pi / 2.0])
    assert np.allclose(states[1], [0.0, 0.0, np.pi / 2.0], atol=1e-8)


def test_rollout_batches_samples():
    controls = np.zeros((4, 5, 3))
    controls[:, :, 0] = 0.2
    states = rollout([0.0, 0.0, 0.0], controls, dt=0.1)

    assert states.shape == (4, 5, 3)
    assert np.allclose(states[:, -1, 0], 0.1)
