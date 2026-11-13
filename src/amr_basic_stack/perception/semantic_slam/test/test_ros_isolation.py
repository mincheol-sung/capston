"""Verify the integration fixture isolates DDS before rclpy initialization."""

import os

import pytest
import test_ros_pipeline


@pytest.mark.parametrize(
    'domain, expected', [(None, '83'), ('', '83'), ('13', '83'), ('84', '84')])
def test_pipeline_isolates_before_init(tmp_path, request, monkeypatch, domain, expected):
    if domain is None:
        monkeypatch.delenv('ROS_DOMAIN_ID', raising=False)
    else:
        monkeypatch.setenv('ROS_DOMAIN_ID', domain)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET')
    monkeypatch.setenv('ROS_STATIC_PEERS', '192.0.2.1')
    monkeypatch.setenv('CYCLONEDDS_URI', '<external-config/>')

    class StopBeforeInit(Exception):
        pass

    def check_init(**_):
        assert os.environ['ROS_DOMAIN_ID'] == expected
        assert os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] == 'LOCALHOST'
        assert os.environ['ROS_STATIC_PEERS'] == ''
        assert os.environ['CYCLONEDDS_URI'] == ''
        raise StopBeforeInit

    monkeypatch.setattr(test_ros_pipeline.rclpy, 'init', check_init)
    fixture = test_ros_pipeline.pipeline.__wrapped__(tmp_path, request, monkeypatch)
    with pytest.raises(StopBeforeInit):
        next(fixture)
