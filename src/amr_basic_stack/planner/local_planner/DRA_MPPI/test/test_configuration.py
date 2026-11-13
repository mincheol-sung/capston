from pathlib import Path

import yaml


def test_controller_period_matches_one_mppi_warm_start_step():
    config_path = Path(__file__).parents[1] / 'config' / 'dra_mppi.yaml'
    parameters = yaml.safe_load(config_path.read_text())['dra_mppi_node'][
        'ros__parameters'
    ]

    controller_period = 1.0 / float(parameters['controller_frequency'])
    assert controller_period == float(parameters['mppi.dt'])


def test_guide_budget_is_independent_of_standard_budget():
    config_path = Path(__file__).parents[1] / 'config' / 'dra_mppi.yaml'
    p = yaml.safe_load(config_path.read_text())['dra_mppi_node']['ros__parameters']
    assert p['mppi.guide_num_rollouts'] == 500
    assert p['dynamic_risk.mode'] == 'monte_carlo'
    assert p['dynamic_risk.final_mode'] == 'gaussian_disk'
    assert p['dynamic_risk.max_obstacles'] == 20
    assert p['mppi.guide_angles_deg'] == [0.0, 45.0, 90.0, -45.0, -90.0]
    assert p['mppi.num_rollouts'] == 321
    assert p['traversability.unknown_is_obstacle'] is True
    assert p['dynamic_risk.guide_parallel_workers'] == 2
    assert p['dynamic_risk.guide_mc_sample_density'] == 992.0
    assert p['dynamic_risk.guide_mc_samples_min'] == 1024
    assert p['dynamic_risk.guide_mc_samples_max'] == 8192
    assert p['dynamic_risk.final_mc_sample_density'] == 992.0
    assert p['dynamic_risk.final_mc_samples_min'] == 2048
    assert p['dynamic_risk.final_mc_samples_max'] == 32768


def test_static_cost_is_explicitly_switchable():
    config_path = Path(__file__).parents[1] / 'config' / 'dra_mppi.yaml'
    p = yaml.safe_load(config_path.read_text())['dra_mppi_node']['ros__parameters']

    assert isinstance(p['static_cost.enabled'], bool)
    assert p['static_cost.enabled'] is True
    assert p['static_cost.soft_weight'] >= 0.0
    assert p['static_cost.decay'] > 0.0


def test_observation_cost_is_optional_and_defaults_off():
    config_path = Path(__file__).parents[1] / 'config' / 'dra_mppi.yaml'
    p = yaml.safe_load(config_path.read_text())['dra_mppi_node']['ros__parameters']
    assert p['observation_cost.enabled'] is False
    assert p['observation_cost.weight'] >= 0.
    assert 0. < p['observation_cost.half_fov_deg'] <= 180.
    assert p['observation_cost.target_timeout'] > 0.
