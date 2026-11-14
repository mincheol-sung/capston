"""No real ip/sudo/slcand or CAN operations: exercise launch setup with fake commands."""
import os
from pathlib import Path
import subprocess
from types import SimpleNamespace

import pytest

from amr_mpc.ak40.setup import prepare_can

SCRIPT = Path(__file__).parents[1] / 'tools/setup_can.sh'


def test_prepare_can_uses_noninteractive_argv(monkeypatch):
    calls = []
    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout='can0 UP', stderr='')
    monkeypatch.setattr(subprocess, 'run', run)
    assert prepare_can(SCRIPT, 'can0', '/dev/serial/by-id/CANable') == 'can0 UP'
    assert calls[0][0] == ['bash', str(SCRIPT), '--non-interactive', '--interface', 'can0', '/dev/serial/by-id/CANable']
    assert calls[0][1]['timeout'] == 10


@pytest.mark.parametrize('failure', ['permission', 'timeout'])
def test_prepare_can_failure_prevents_start(monkeypatch, failure):
    def run(command, **kwargs):
        if failure == 'timeout':
            raise subprocess.TimeoutExpired(command, 10)
        return SimpleNamespace(returncode=1, stdout='', stderr='sudo: a password is required')
    monkeypatch.setattr(subprocess, 'run', run)
    with pytest.raises(RuntimeError, match='not started'):
        prepare_can(SCRIPT, 'can0')


@pytest.fixture
def commands(tmp_path):
    bindir = tmp_path / 'bin'
    bindir.mkdir()
    state = tmp_path / 'state'
    log = tmp_path / 'commands'
    programs = {
        'ip': '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
p=Path(os.environ['FAKE_STATE'])
a=sys.argv[1:]
if 'set' in a:
    if os.environ.get('FAIL_UP') == '1': sys.exit(1)
    p.write_text('UP'); sys.exit(0)
if not p.exists(): sys.exit(1)
if '-j' in a: print(json.dumps([{'flags': [p.read_text()]}]))
else: print('can0 ' + p.read_text())
''',
        'slcand': '''#!/bin/bash
printf '%s\\n' "$*" >> "$FAKE_LOG"
echo DOWN > "$FAKE_STATE"
''',
        'modprobe': '#!/bin/bash\nexit 0\n',
        'sudo': '#!/bin/bash\nif [[ "$1" == "-n" ]]; then shift; fi\nexec "$@"\n',
    }
    for name, source in programs.items():
        p = bindir / name
        p.write_text(source)
        p.chmod(0o755)
    env = dict(os.environ, PATH=str(bindir) + ':' + os.environ['PATH'], FAKE_STATE=str(state), FAKE_LOG=str(log))
    return state, log, env


def execute(env, *args):
    return subprocess.run(['bash', str(SCRIPT), '--non-interactive', *args], env=env, capture_output=True, text=True)


def test_existing_up_needs_no_usb_device(commands):
    state, log, env = commands
    state.write_text('UP')
    result = execute(env, '/nonexistent-device')
    assert result.returncode == 0, result.stderr
    assert not log.exists()


def test_existing_down_is_brought_up(commands):
    state, log, env = commands
    state.write_text('DOWN')
    result = execute(env)
    assert result.returncode == 0, result.stderr
    assert state.read_text() == 'UP'
    assert not log.exists()


def test_missing_interface_creates_named_slcan(commands):
    state, log, env = commands
    result = execute(env, '--interface', 'can7', '/dev/null')
    assert result.returncode == 0, result.stderr
    assert '-o -c -s8 /dev/null can7' in log.read_text()
    assert state.read_text() == 'UP'


def test_failed_activation_returns_failure(commands):
    state, log, env = commands
    state.write_text('DOWN')
    result = execute(dict(env, FAIL_UP='1'))
    assert result.returncode != 0
    assert state.read_text() == 'DOWN'


def test_device_check_does_not_activate_can(commands):
    state, log, env = commands
    result = execute(env, '--check', '/dev/null')
    assert result.returncode == 0
    assert not state.exists() and not log.exists()


def test_real_launch_does_not_start_nodes_if_setup_fails(monkeypatch):
    import importlib.util
    pytest.importorskip('launch_ros')
    launch_path = Path(__file__).parents[1] / 'launch/torque_experiment.launch.py'
    spec = importlib.util.spec_from_file_location('torque_launch_setup_test', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    values = dict(plant='real', state_source='estimator', slam_mode='scan_time',
                  state_topic='', auto_setup_can='true', can_interface='', can_device='',
                  armed='false')   # R10: 실차 즉시 무장 금지 검사가 CAN 준비 전에 armed 를 읽는다
    class Configuration:
        def __init__(self, key): self.key = key
        def perform(self, context): return values[self.key]
    monkeypatch.setattr(module, 'LaunchConfiguration', Configuration)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda _: str(launch_path.parents[1]))
    def fail(*args):
        raise RuntimeError('CAN setup failed; motor nodes were not started')
    monkeypatch.setattr(module, 'prepare_can', fail)
    monkeypatch.setattr(module, 'Node', lambda **kw: pytest.fail('node constructed before CAN setup succeeded'))
    with pytest.raises(RuntimeError, match='not started'):
        module._nodes(None)
