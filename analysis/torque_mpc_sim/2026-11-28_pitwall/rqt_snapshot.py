#!/usr/bin/env python3
"""rqt_reconfigure 화면을 화면 없이 (Qt offscreen) 띄워 PNG 로 찍고, 사용자가 칸에 값을 넣는 것을 흉내 낸다 (2026-11-28 pitwall 확인용).

  QT_QPA_PLATFORM=offscreen python3 rqt_snapshot.py OUT_DIR
시뮬 스택 (도메인 97) 이 떠 있어야 한다. 하는 일:
  1. figure8_reference_node 를 골라 화면 저장 (rqt_ref.png)
  2. speed 칸에 1.5 를 넣음 → 부하 예산에 걸려 거부 → 칸이 원래 값으로 돌아오는지 (revert)
  3. accel 칸에 0.6 을 넣음 → 받음
  4. preset 칸에 circ → 프리셋 적용 뒤 칸이 비고 shape·width·speed 가 바뀌는지
  5. torque_mpc_node 화면 저장 (rqt_mpc.png), slip_fb_scale 9 (범위 밖) → 거부·되돌림, 0.5 → 받음
"""
import os
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from python_qt_binding.QtWidgets import QApplication

from rqt_reconfigure.param_widget import ParamWidget

OUT = sys.argv[1] if len(sys.argv) > 1 else "."


class Ctx:
    def __init__(self, node):
        self.node = node

    def argv(self):
        return []

    def serial_number(self):
        return 1


def pump(app, sec):
    t = time.time() + sec
    while time.time() < t:
        app.processEvents()
        time.sleep(0.02)


def editors(w, grn):
    return w._param_edit_widget._param_client_widgets[grn]._editor_widgets


def text_of(ed):
    for attr in ("_paramval_lineEdit", "_paramval_lineedit"):
        if hasattr(ed, attr):
            return getattr(ed, attr).text()
    if hasattr(ed, "_checkbox"):
        return str(ed._checkbox.isChecked())
    return str(ed.parameter.value)


def main():
    app = QApplication(sys.argv)
    rclpy.init()
    node = rclpy.create_node("rqt_snapshot")
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    w = ParamWidget(Ctx(node))
    w.resize(1400, 1900)
    w.show()
    REF, MPC = "/figure8_reference_node", "/torque_mpc_node"
    t_end = time.time() + 20
    while time.time() < t_end and not {REF, MPC} <= set(w._nodesel_widget.get_nodeitems()):
        pump(app, 0.5)
    print("rqt 가 찾은 노드:", sorted(w._nodesel_widget.get_nodeitems()))
    w._nodesel_widget.node_selected(REF)
    pump(app, 3)
    w.grab().save(os.path.join(OUT, "rqt_ref.png"))
    ed = editors(w, REF)
    print("figure8 편집 칸:", len(ed), " read_only 아닌 칸:", sum(1 for e in ed.values() if not e.descriptor.read_only))
    print("speed 범위:", [(r.from_value, r.to_value) for r in ed["speed"].descriptor.floating_point_range])
    before = text_of(ed["speed"])
    ed["speed"].update(1.5)
    shown = text_of(ed["speed"])
    pump(app, 2)
    print(f"speed 1.5 입력 (부하 초과): 입력 직후 칸 {shown} → 2 s 뒤 칸 {text_of(ed['speed'])} (원래 {before})")
    ed["accel"].update(0.6)
    pump(app, 2)
    print(f"accel 0.6 입력: 칸 {text_of(ed['accel'])}, 노드 값 {node_param(node, REF, 'accel')}")
    ed["preset"].update("circ")
    pump(app, 3)
    print(f"preset circ 입력: preset 칸 '{text_of(ed['preset'])}', shape {text_of(ed['shape'])}, width {text_of(ed['width'])}, "
          f"speed {text_of(ed['speed'])}, laps {text_of(ed['laps'])}")
    w.grab().save(os.path.join(OUT, "rqt_ref_after.png"))
    w._nodesel_widget.node_selected(REF)          # 닫기 (같은 노드를 다시 고르면 빠진다)
    pump(app, 1)
    w._nodesel_widget.node_selected(MPC)
    pump(app, 3)
    em = editors(w, MPC)
    w.grab().save(os.path.join(OUT, "rqt_mpc.png"))
    em["slip_fb_scale"].update(9.0)
    pump(app, 2)
    print(f"slip_fb_scale 9 입력 (범위 0~5 밖): 칸 {text_of(em['slip_fb_scale'])}")
    em["slip_fb_scale"].update(0.5)
    pump(app, 2)
    print(f"slip_fb_scale 0.5 입력: 칸 {text_of(em['slip_fb_scale'])}, 노드 값 {node_param(node, MPC, 'slip_fb_scale')}")
    w.close()
    ex.shutdown()
    rclpy.shutdown()


def node_param(node, target, name):
    from rclpy.parameter_client import AsyncParameterClient
    from rclpy.parameter import parameter_value_to_python
    cl = AsyncParameterClient(node, target)
    cl.wait_for_services(timeout_sec=3)
    f = cl.get_parameters([name])
    t = time.time() + 3
    while not f.done() and time.time() < t:
        time.sleep(0.05)
    return parameter_value_to_python(f.result().values[0]) if f.done() else None


if __name__ == "__main__":
    main()
