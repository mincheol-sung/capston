#!/usr/bin/env python3
"""SocketCAN 연결 — 파이썬 내장 socket(AF_CAN) 만 쓴다 (python-can·can-utils 설치 불필요).

실차:  CANable 2.0 (candleLight 펌웨어면)  sudo ip link set can0 up type can bitrate 1000000
       slcan 펌웨어면                     sudo slcand -o -s8 /dev/ttyACM0 can0 && sudo ip link set can0 up
시뮬:  가상 버스                           sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
       (sudo 없이: unshare -rn 으로 만든 네트워크 이름공간 안에서 ip link add … — 그 안에서 ROS 도 같이 돌려야 한다)
"""
import select
import socket
import struct

_FMT = "=IB3x8s"            # struct can_frame: can_id(u32), len(u8), pad×3, data[8]
_SIZE = struct.calcsize(_FMT)


class CanBus:
    def __init__(self, ifname):
        self.ifname = ifname
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((ifname,))
        self.sock.setblocking(False)

    def send(self, can_id, data):
        data = bytes(data)
        self.sock.send(struct.pack(_FMT, can_id & socket.CAN_SFF_MASK, len(data), data.ljust(8, b"\x00")))

    def recv(self, timeout=0.0):
        """프레임 하나 (can_id, data) 또는 None. timeout 초까지 기다린다 (0 이면 기다리지 않음)."""
        if timeout > 0.0:
            r, _, _ = select.select([self.sock], [], [], timeout)
            if not r:
                return None
        try:
            frame = self.sock.recv(_SIZE)
        except BlockingIOError:
            return None
        can_id, n, data = struct.unpack(_FMT, frame)
        if can_id & (socket.CAN_ERR_FLAG | socket.CAN_RTR_FLAG):
            return None                                   # 에러·원격 프레임은 버린다
        return can_id & socket.CAN_SFF_MASK, data[:n]

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
