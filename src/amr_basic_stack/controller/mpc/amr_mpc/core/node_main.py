#!/usr/bin/env python3
"""Python 노드의 spin 과 종료 신호 처리 (R9 리뷰 2026-11-14).

왜 필요한가
  노드들은 rclpy.init(signal_handler_options=SignalHandlerOptions.NO) 로 rclpy 의 신호 처리를 끈다 — Ctrl+C 뒤에도
  마지막 메시지(DAMPING 등)를 낼 수 있게 (R6). 그러면 파이썬 기본 동작으로 Ctrl+C 가 KeyboardInterrupt 가 되는데,
  그 신호가 rclpy 가 메시지를 파이썬 객체로 바꾸는 C++(pybind11) 코드 한가운데 떨어지면 KeyboardInterrupt 가
  "RuntimeError: Unable to convert call argument '0' to Python object" 로 바뀌어 나온다.
  `except KeyboardInterrupt` 가 못 잡아 Traceback 과 함께 exit code 1 로 죽는다
  (fault_monitor 에서 20 판 중 4 판, 메시지가 많이 오는 노드일수록 잦다).
  또 SIGTERM(kill·systemd·launch 의 2차 신호)·SIGHUP(터미널 닫힘)은 파이썬 기본 동작이 즉시 종료라 finally 가 안 돈다.

spin(node) 이 하는 일
  - SIGINT·SIGTERM·SIGHUP 을 받으면 "멈추라는 신호가 왔다" 를 기록하고 KeyboardInterrupt 를 일으킨다
  - 그 뒤에 나오는 RuntimeError 는 종료로 본다. 신호가 없었는데 난 RuntimeError 는 진짜 오류라 그대로 올린다
  - 돌아오면 세 신호를 무시로 바꾼다 — 호출한 쪽의 정리(finally: 마지막 DAMPING, 요약 저장 등)가 두 번째 신호에 끊기지 않게
"""
import signal

import rclpy

STOP_SIGNALS = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)


def spin(node):
    """node 를 종료 신호가 올 때까지 돌린다. 반환 True = 신호로 끝남."""
    stopping = []

    def on_signal(signum, _frame):
        stopping.append(signum)
        raise KeyboardInterrupt

    for s in STOP_SIGNALS:
        signal.signal(s, on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if not stopping:
            raise
    finally:
        for s in STOP_SIGNALS:
            signal.signal(s, signal.SIG_IGN)
    return bool(stopping)
