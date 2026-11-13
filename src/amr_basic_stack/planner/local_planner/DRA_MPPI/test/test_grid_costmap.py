import numpy as np

from dra_mppi.grid_costmap import DistanceField, GridMetadata


def test_distance_field_detects_occupied_and_outside_points():
    metadata = GridMetadata(
        width=5,
        height=5,
        resolution=1.0,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
    )
    occupancy = np.zeros((5, 5), dtype=np.int16)
    occupancy[2, 2] = 100
    field = DistanceField.from_occupancy(occupancy.ravel(), metadata)

    clearance = field.query(np.array([[2.5, 2.5], [0.5, 0.5], [-1.0, 0.0]]))
    assert clearance[0] == 0.0
    assert clearance[1] > 0.0
    assert clearance[2] == 0.0


def test_trajectory_clearance_checks_between_rollout_states():
    metadata = GridMetadata(
        width=30,
        height=30,
        resolution=0.1,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
    )
    occupancy = np.zeros((30, 30), dtype=np.int16)
    occupancy[10, 10] = 100
    field = DistanceField.from_occupancy(occupancy.ravel(), metadata)

    # The endpoints are free, but the swept segment crosses the occupied cell.
    positions = np.array([[[1.55, 1.05]]])
    clearance = field.trajectory_clearances(
        positions,
        initial_xy=np.array([0.55, 1.05]),
    )

    assert clearance.shape == (1, 1)
    assert clearance[0, 0] == 0.0
