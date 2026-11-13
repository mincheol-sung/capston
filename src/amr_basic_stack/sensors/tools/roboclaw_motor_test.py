#!/usr/bin/env python3
"""roboclaw_motor_test.py — 채널별 모터 구동 테스트. 바퀴를 반드시 띄운 상태에서.

한 채널씩만, 낮은 속도로, 1초씩 돌린다. 매 단계 끝과 어떤 예외 상황에서도
전 채널에 정지 명령을 보낸다.

두 가지를 순서대로 시도해서 원인을 분리한다.
  1. SpeedM1/M2 (35/36)  — 엔코더 폐루프. 드라이버가 실제로 쓰는 명령.
  2. DutyCycle (32/33)   — 개루프. 엔코더가 없어도 돈다.

1은 안 되는데 2는 되면  -> 엔코더/속도PID 문제
둘 다 안 되면            -> 모터 배선/전원/보드 출력단 또는 S4 E-Stop
둘 다 되면               -> 보드는 정상. ROS 쪽 파라미터 문제
"""

import struct
import sys
import time

import serial

SPEED_M1, SPEED_M2 = 35, 36
DUTY_M1, DUTY_M2 = 32, 33
GETM1ENC, GETM2ENC = 16, 17

TEST_QPPS = 1000        # 보드 최대 QPPS 약 9900 의 ~10%
TEST_DUTY = 4915        # 32767 의 15%
RUN_SECONDS = 1.0

# base.yaml 의 출력 매핑 그대로
CHANNELS = [
    ("rear_left",   "/dev/ttyACM0", 128, SPEED_M1, DUTY_M1, GETM1ENC),
    ("front_left",  "/dev/ttyACM0", 128, SPEED_M2, DUTY_M2, GETM2ENC),
    ("front_right", "/dev/ttyACM1", 129, SPEED_M1, DUTY_M1, GETM1ENC),
    ("rear_right",  "/dev/ttyACM1", 129, SPEED_M2, DUTY_M2, GETM2ENC),
]


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def write_cmd(ser, addr, cmd, payload: bytes) -> bool:
    packet = bytes([addr & 0xFF, cmd & 0xFF]) + payload
    ser.reset_input_buffer()
    ser.write(packet + struct.pack(">H", crc16(packet)))
    return ser.read(1) == b"\xff"


def read_enc(ser, addr, cmd):
    ser.reset_input_buffer()
    packet = bytes([addr & 0xFF, cmd & 0xFF])
    ser.write(packet)
    resp = ser.read(7)
    if len(resp) != 7:
        return None
    if struct.unpack(">H", resp[5:])[0] != crc16(packet + resp[:5]):
        return None
    return struct.unpack(">i", resp[:4])[0]


def stop_everything(sers):
    """어떤 경로로 끝나든 반드시 호출된다. 속도·듀티 양쪽으로 0 을 박는다."""
    for port, addr in (("/dev/ttyACM0", 128), ("/dev/ttyACM1", 129)):
        ser = sers.get(port)
        if ser is None:
            continue
        for cmd, payload in (
            (SPEED_M1, struct.pack(">i", 0)), (SPEED_M2, struct.pack(">i", 0)),
            (DUTY_M1, struct.pack(">h", 0)), (DUTY_M2, struct.pack(">h", 0)),
        ):
            try:
                write_cmd(ser, addr, cmd, payload)
            except Exception:
                pass


def run_one(ser, addr, cmd, payload, enc_cmd, label):
    before = read_enc(ser, addr, enc_cmd)
    ack = write_cmd(ser, addr, cmd, payload)
    t0 = time.time()
    peak = 0
    while time.time() - t0 < RUN_SECONDS:
        time.sleep(0.1)
        now = read_enc(ser, addr, enc_cmd)
        if now is not None and before is not None:
            peak = max(peak, abs(now - before))
    # 정지 먼저, 그 다음 최종 측정
    write_cmd(ser, addr, SPEED_M1 if cmd in (SPEED_M1, DUTY_M1) else SPEED_M2,
              struct.pack(">i", 0))
    write_cmd(ser, addr, DUTY_M1 if cmd in (SPEED_M1, DUTY_M1) else DUTY_M2,
              struct.pack(">h", 0))
    time.sleep(0.3)
    after = read_enc(ser, addr, enc_cmd)
    delta = (after - before) if (after is not None and before is not None) else None
    moved = delta is not None and abs(delta) > 20
    print(f"      {label:8s} ACK={'O' if ack else 'X'}  "
          f"엔코더 {before} -> {after}  Δ={delta}  "
          f"{'✅ 움직임' if moved else '❌ 정지'}")
    return moved


def main():
    print("=" * 64)
    print("모터 구동 테스트 — 바퀴가 바닥에서 떠 있어야 한다.")
    print(f"채널당 {RUN_SECONDS}초, 폐루프 {TEST_QPPS} qpps / 개루프 듀티 15%")
    print("=" * 64)

    sers = {}
    results = {}
    try:
        for port in ("/dev/ttyACM0", "/dev/ttyACM1"):
            sers[port] = serial.Serial(port=port, baudrate=115200, timeout=0.15)
        stop_everything(sers)
        time.sleep(0.3)

        for name, port, addr, speed_cmd, duty_cmd, enc_cmd in CHANNELS:
            ser = sers[port]
            print(f"\n  [{name}]  {port} addr={addr}")
            closed = run_one(ser, addr, speed_cmd, struct.pack(">i", TEST_QPPS),
                             enc_cmd, "폐루프")
            time.sleep(0.5)
            open_loop = run_one(ser, addr, duty_cmd, struct.pack(">h", TEST_DUTY),
                                enc_cmd, "개루프")
            time.sleep(0.5)
            results[name] = (closed, open_loop)
    except KeyboardInterrupt:
        print("\n중단됨 — 정지 명령 전송")
    except Exception as e:
        print(f"\n오류: {e} — 정지 명령 전송")
    finally:
        stop_everything(sers)
        for s in sers.values():
            s.close()

    print("\n" + "=" * 64)
    print(f"  {'채널':12s} {'폐루프(Speed)':>14s} {'개루프(Duty)':>14s}")
    for name, (c, o) in results.items():
        print(f"  {name:12s} {'O' if c else 'X':>14s} {'O' if o else 'X':>14s}")
    print("=" * 64)

    if not results:
        return
    any_closed = any(c for c, _ in results.values())
    any_open = any(o for _, o in results.values())
    if not any_closed and not any_open:
        print("결론: 어느 채널도 안 돈다 -> 모터 배선/전원 또는 S4 E-Stop 차단")
    elif not any_closed and any_open:
        print("결론: 개루프만 된다 -> 엔코더 미연결 또는 속도PID 미설정")
    elif all(c for c, _ in results.values()):
        print("결론: 보드는 정상. 문제는 ROS 파라미터 쪽")
    else:
        dead = [n for n, (c, o) in results.items() if not c]
        print(f"결론: 일부 채널만 폐루프 실패 -> {dead} 의 엔코더 확인")


if __name__ == "__main__":
    sys.exit(main())
