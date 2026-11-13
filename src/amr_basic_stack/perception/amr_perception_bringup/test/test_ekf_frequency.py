"""두 EKF 의 출력 주기가 짝을 이루는지 지킨다. ROS 없이 돈다.

왜 짝이어야 하는가: ekf_carto 는 world_frame 이 map 이라 자기 상태를 그대로
내지 못하고

    map->odom = (map->base, 자기 상태) ∘ (odom->base, TF 조회)⁻¹

로 만든다. 그 odom->base 는 ekf_filter_node 가 내는 TF 다. 주기가 다르면
ekf_carto 가 최대 (1/느린 쪽 주기) 만큼 묵은 값을 빼고, 그 시차가

    병진 |v|·Δt        회전 |wz|·Δt·r

만큼의 위치 오차가 되어 map->odom 에 실린다.

2026-09-14 실측 (bag mpc_chain_20260914_155111, 108.9 s, 30 vs 50 Hz):
    map->odom 누적경로 35.87 m / 순변위 0.23 m = 비 156.2
    같은 stamp 로 보간해 재계산하면 16.02 m = 비 69.7
    -> 떨림의 55.3 % 가 주기 불일치였다.

ekf_filter_node 는 모드와 무관하게 하나이고 ekf_carto 는 모드별로 셋이므로,
한쪽만 고치면 다른 모드가 조용히 깨진다. 실제로 그런 적이 있다.
"""

import pathlib

import pytest

import yaml

CONFIG = pathlib.Path(__file__).resolve().parent.parent / 'config'
CARTO_CONFIGS = (
    'ekf_carto_mapping.yaml',
    'ekf_carto_localization.yaml',
    'ekf_carto_slam.yaml',
)


def _params(name, node):
    return yaml.safe_load((CONFIG / name).read_text())[node]['ros__parameters']


def test_filter_ekf_publishes_the_odom_level_tf():
    """이 노드가 odom->base_footprint 의 주인이다. 그래서 짝의 기준이 된다."""
    p = _params('ekf.yaml', 'ekf_filter_node')
    assert p['world_frame'] == p['odom_frame'] == 'odom'
    assert p['publish_tf'] is True


@pytest.mark.parametrize('name', CARTO_CONFIGS)
def test_carto_ekf_publishes_the_map_level_tf(name):
    """world_frame 이 map 이면 robot_localization 은 map->odom 을 낸다."""
    p = _params(name, 'ekf_carto')
    assert p['world_frame'] == p['map_frame'] == 'map'
    assert p['publish_tf'] is True


@pytest.mark.parametrize('name', CARTO_CONFIGS)
def test_frequencies_match(name):
    """한쪽만 바꾸면 그 모드의 map->odom 이 시차만큼 떤다."""
    base = _params('ekf.yaml', 'ekf_filter_node')['frequency']
    carto = _params(name, 'ekf_carto')['frequency']
    assert carto == base, (
        f'{name} 의 frequency {carto} 가 ekf.yaml 의 {base} 와 다르다. '
        f'시차 {abs(1.0 / carto - 1.0 / base) * 1000:.1f} ms 가 map->odom 에 '
        f'|v|·Δt 와 |wz|·Δt·r 만큼의 오차로 실린다.'
    )


@pytest.mark.parametrize('name', ('ekf.yaml',) + CARTO_CONFIGS)
def test_sensor_timeout_covers_several_cycles(name):
    """한 주기보다 짧은 timeout 은 매 주기 predict-only 로 빠지게 만든다."""
    node = 'ekf_filter_node' if name == 'ekf.yaml' else 'ekf_carto'
    p = _params(name, node)
    assert p['sensor_timeout'] * p['frequency'] >= 2.0


@pytest.mark.parametrize('name', CARTO_CONFIGS)
def test_base_link_frame_is_shared(name):
    """두 EKF 가 서로 다른 base 를 쓰면 map->odom 계산이 통째로 어긋난다."""
    base = _params('ekf.yaml', 'ekf_filter_node')['base_link_frame']
    assert _params(name, 'ekf_carto')['base_link_frame'] == base
