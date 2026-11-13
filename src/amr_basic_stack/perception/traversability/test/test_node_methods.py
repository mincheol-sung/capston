"""노드가 호출하는 self._메서드 가 실제로 정의돼 있는지 정적으로 확인한다.

rclpy 노드는 콜백 안에서 터지면 스택트레이스가 로그로만 나가고 토픽이 조용히
멈춘다. 그래서 "메서드 이름을 잘못 썼다" 같은 실수가 통합 테스트에서
"수신 실패" 로만 보이고 원인이 안 보인다. 여기서 먼저 걸러낸다.

(2026-09-09 실제로 이 실수를 했다 — _log_diagnostics 호출만 넣고 정의를
 다른 파일의 앵커에 붙이려다 조용히 실패했다.)
"""

import ast
import pathlib

import pytest

PACKAGE = pathlib.Path(__file__).resolve().parent.parent / 'traversability'
MODULES = sorted(PACKAGE.glob('*_node.py'))


def _self_attribute_calls(class_node):
    """클래스 안에서 self.<name>(...) 형태로 호출되는 이름들."""
    called = set()
    for node in ast.walk(class_node):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if (isinstance(func, ast.Attribute)
                and isinstance(func.value, ast.Name)
                and func.value.id == 'self'):
            called.add(func.attr)
    return called


def _class_defs(module_path):
    tree = ast.parse(module_path.read_text())
    return [n for n in tree.body if isinstance(n, ast.ClassDef)]


@pytest.mark.parametrize('module_path', MODULES, ids=lambda p: p.name)
def test_every_called_private_method_is_defined(module_path):
    for class_node in _class_defs(module_path):
        defined = {
            n.name for n in class_node.body
            if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        called = _self_attribute_calls(class_node)
        # rclpy.Node 에서 물려받는 것은 밑줄로 시작하지 않는다.
        private = {name for name in called if name.startswith('_')}
        missing = sorted(private - defined)
        assert not missing, (
            f'{module_path.name}:{class_node.name} 이 정의되지 않은 메서드를 '
            f'호출한다: {missing}'
        )


def test_modules_were_actually_found():
    """파일 목록이 비면 위 테스트가 아무것도 검사하지 않고 통과한다."""
    assert len(MODULES) >= 2, [p.name for p in MODULES]
