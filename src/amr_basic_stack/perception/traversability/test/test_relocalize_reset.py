"""/initialpose 재측위 리셋의 배선을 정적으로 확인한다.

이 기능은 "안 되면 조용히 안 되는" 종류다 — 구독이 빠지거나 QoS 가 안 맞으면
메시지가 한 개도 안 오는데 노드는 정상으로 보이고, 격자만 옛 자세의 FREE 셀을
영구히 물고 있게 된다(free_memory_seconds: 0.0). 그래서 실차에서 확인하기 전에
여기서 걸러낸다.

특히 QoS: amcl_bootstrap_node 는 /initialpose 를 BEST_EFFORT 로 낸다
(set_initial_pose_node 구독자에 맞춘 것). RELIABLE 구독자는 BEST_EFFORT
발행자와 매칭되지 않으므로, 여기서 RELIABLE 을 쓰면 AMCL 자동 초기위치가
통째로 안 들어온다.
"""

import ast
import pathlib

import pytest

NODE = (pathlib.Path(__file__).resolve().parent.parent
        / 'traversability' / 'traversability_node.py')


@pytest.fixture(scope='module')
def tree():
    return ast.parse(NODE.read_text())


def _function(tree, name):
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    return None


def _subscription_calls(tree):
    """self.create_subscription(...) 호출 노드들."""
    calls = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == 'create_subscription'):
            calls.append(node)
    return calls


def test_methods_exist(tree):
    for name in ('_on_initial_pose', '_reset_memory', '_settling'):
        assert _function(tree, name) is not None, f'{name} 가 정의돼 있지 않다'


def test_initial_pose_is_subscribed(tree):
    """콜백이 실제로 create_subscription 에 넘겨져야 한다."""
    handlers = []
    for call in _subscription_calls(tree):
        for arg in call.args:
            if (isinstance(arg, ast.Attribute)
                    and isinstance(arg.value, ast.Name)
                    and arg.value.id == 'self'):
                handlers.append(arg.attr)
    assert '_on_initial_pose' in handlers, \
        '_on_initial_pose 가 어떤 구독에도 연결돼 있지 않다'


def test_initial_pose_subscription_is_best_effort(tree):
    """BEST_EFFORT 여야 amcl_bootstrap_node 의 발행을 받는다."""
    target = None
    for call in _subscription_calls(tree):
        if any(isinstance(a, ast.Attribute) and a.attr == '_on_initial_pose'
               for a in call.args):
            target = call
    assert target is not None
    source = ast.dump(target)
    assert 'BEST_EFFORT' in source, (
        '/initialpose 구독이 BEST_EFFORT 가 아니다 — amcl_bootstrap_node 의 '
        'BEST_EFFORT 발행과 매칭되지 않아 한 개도 못 받는다')


def test_reset_is_called_from_both_paths(tree):
    """기하 변경과 재측위 양쪽에서 같은 리셋을 써야 한다."""
    for caller in ('_on_base_map', '_on_initial_pose'):
        func = _function(tree, caller)
        assert func is not None
        called = {n.func.attr for n in ast.walk(func)
                  if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute)}
        assert '_reset_memory' in called, f'{caller} 가 _reset_memory 를 안 부른다'


@pytest.mark.parametrize('callback', ['_on_scan', '_on_band_cloud'])
def test_accumulating_callbacks_are_gated(tree, callback):
    """정착 구간에는 누적 콜백이 일찍 빠져나가야 한다."""
    func = _function(tree, callback)
    assert func is not None
    called = {n.func.attr for n in ast.walk(func)
              if isinstance(n, ast.Call)
              and isinstance(n.func, ast.Attribute)}
    assert '_settling' in called, \
        f'{callback} 에 재측위 정착 가드가 없다 — 옛 좌표로 다시 쌓인다'


def test_settle_seconds_is_a_parameter(tree):
    """정착 시간이 하드코딩이 아니라 파라미터여야 한다."""
    declared = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == '_declare'
                and node.args
                and isinstance(node.args[0], ast.Constant)):
            declared.add(node.args[0].value)
    assert 'relocalize_settle_seconds' in declared
    assert 'initialpose_topic' in declared
