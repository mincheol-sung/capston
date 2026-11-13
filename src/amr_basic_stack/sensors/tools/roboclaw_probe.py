#!/usr/bin/env python3
"""roboclaw_probe.py — RoboClaw 통신 상태를 직접 진단한다. 읽기 전용.

모터를 절대 돌리지 않는다. 읽기 명령만 보낸다.

확인하는 것
  1. 어떤 baudrate/주소 조합에서 응답하는가  (펌웨어 버전 읽기)
  2. 메인 배터리 전압                        (전원이 들어와 있는가)
  3. 엔코더 카운트                            (엔코더가 물려 있는가)
  4. 속도 PID 의 QPPS 설정값                  (SpeedM1/M2 를 쓸 수 있는 상태인가)

4번이 중요하다. 드라이버는 SpeedM1/M2(35/36)를 쓰는데 이건 엔코더 기반
폐루프 명령이다. 엔코더가 없거나 속도 PID 가 설정 안 돼 있으면 명령이
정상 전달돼도 바퀴가 안 돈다.

실행 전에 ros2 launch 를 반드시 종료할 것. 포트를 동시에 못 연다.
  python3 roboclaw_probe.py
"""

import struct
import sys

try:
    import serial
except ImportError:
    sys.exit("python3-serial 이 없다: sudo apt install python3-serial")

PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUDS = [38400, 115200, 57600, 230400, 460800]
ADDRESSES = [128, 129, 130, 131, 132, 133, 134, 135]

GETVERSION = 21
GETMBATT = 24
GETM1ENC = 16
GETM2ENC = 17
READM1PID = 55
READM2PID = 56


def crc16(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ask(ser, address, command, nbytes):
    """읽기 명령을 보내고 nbytes + CRC2 를 받는다. CRC 검증까지 한다."""
    ser.reset_input_buffer()
    packet = bytes([address & 0xFF, command & 0xFF])
    ser.write(packet)
    resp = ser.read(nbytes + 2)
    if len(resp) != nbytes + 2:
        return None
    data = resp[:nbytes]
    got = struct.unpack(">H", resp[nbytes:])[0]
    if got != crc16(packet + data):
        return None
    return data


def read_version(ser, address):
    """버전은 가변길이 문자열이라 별도 처리한다."""
    ser.reset_input_buffer()
    packet = bytes([address & 0xFF, GETVERSION])
    ser.write(packet)
    raw = ser.read(48)
    if b"\x00" not in raw:
        return None
    end = raw.index(b"\x00") + 1
    body, rest = raw[:end], raw[end:]
    if len(rest) < 2:
        return None
    if struct.unpack(">H", rest[:2])[0] != crc16(packet + body):
        return None
    return body[:-1].decode(errors="replace").strip()


def probe_port(port):
    print(f"\n{'='*62}\n{port}\n{'='*62}")
    found = []
    for baud in BAUDS:
        try:
            ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
        except Exception as e:
            print(f"  포트 열기 실패: {e}")
            return found
        with ser:
            for addr in ADDRESSES:
                ver = read_version(ser, addr)
                if ver:
                    print(f"  ✅ 응답  baud={baud}  address={addr}")
                    print(f"     펌웨어: {ver}")
                    found.append((baud, addr))
                    details(ser, addr)
    if not found:
        print("  ❌ 어떤 baudrate/주소 조합에서도 응답이 없다.")
        print("     → 보드가 Packet Serial 모드가 아니거나, 배선/포트가 다르다.")
    return found


def details(ser, addr):
    batt = ask(ser, addr, GETMBATT, 2)
    if batt:
        v = struct.unpack(">H", batt)[0] / 10.0
        mark = "⚠️ 모터 전원 없음" if v < 6.0 else "정상"
        print(f"     메인 배터리: {v:.1f} V  ({mark})")
    else:
        print("     메인 배터리: 읽기 실패")

    for name, cmd in (("M1", GETM1ENC), ("M2", GETM2ENC)):
        enc = ask(ser, addr, cmd, 5)
        if enc:
            count = struct.unpack(">i", enc[:4])[0]
            print(f"     {name} 엔코더 카운트: {count}")
        else:
            print(f"     {name} 엔코더: 읽기 실패 (엔코더 미연결 가능)")

    for name, cmd in (("M1", READM1PID), ("M2", READM2PID)):
        pid = ask(ser, addr, cmd, 16)
        if pid:
            p, i, d, qpps = struct.unpack(">IIII", pid)
            flag = "  ⚠️ QPPS=0 → 속도 PID 미설정. SpeedM1/M2 로는 안 움직인다" if qpps == 0 else ""
            print(f"     {name} 속도PID: P={p/65536:.2f} I={i/65536:.2f} "
                  f"D={d/65536:.2f} QPPS={qpps}{flag}")
        else:
            print(f"     {name} 속도PID: 읽기 실패")


def watch():
    """손으로 바퀴를 굴리면서 엔코더가 반응하는지 본다. 읽기 전용.

    카운트가 안 변하는 채널은 엔코더가 안 물려 있는 것이다.
    그 채널은 SpeedM1/M2(폐루프)로는 정상 구동되지 않는다.
    """
    wheels = {  # base.yaml 의 출력 매핑 그대로
        ("/dev/ttyACM0", 128, GETM1ENC): "rear_left   (left_m1)",
        ("/dev/ttyACM0", 128, GETM2ENC): "front_left  (left_m2)",
        ("/dev/ttyACM1", 129, GETM1ENC): "front_right (right_m1)",
        ("/dev/ttyACM1", 129, GETM2ENC): "rear_right  (right_m2)",
    }
    sers = {p: serial.Serial(port=p, baudrate=115200, timeout=0.1)
            for p in {k[0] for k in wheels}}
    base = {}
    print("바퀴를 하나씩 손으로 굴려보세요. Ctrl+C 로 종료.\n")
    try:
        while True:
            line = []
            for (port, addr, cmd), label in wheels.items():
                enc = ask(sers[port], addr, cmd, 5)
                if enc is None:
                    line.append(f"{label}: 읽기실패")
                    continue
                v = struct.unpack(">i", enc[:4])[0]
                base.setdefault((port, cmd), v)
                delta = v - base[(port, cmd)]
                flag = "  ← 움직임!" if abs(delta) > 5 else ""
                line.append(f"{label}: {v:>9d} (Δ{delta:>+7d}){flag}")
            print("\033[2J\033[H" + "\n".join(line), flush=True)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        for s in sers.values():
            s.close()


def main():
    if "--watch" in sys.argv:
        watch()
        return

    print("RoboClaw 진단 — 읽기 전용, 모터를 돌리지 않는다.")
    print("ros2 launch 가 떠 있으면 먼저 종료할 것.")
    all_found = []
    for port in PORTS:
        all_found += probe_port(port)

    print(f"\n{'='*62}")
    if not all_found:
        print("결론: 두 포트 모두 무응답.")
        print("  - BasicMicro Motion Studio 로 보드를 Packet Serial 모드로 설정했는지")
        print("  - USB 케이블이 데이터용인지 (충전 전용 케이블 주의)")
    else:
        bauds = {b for b, _ in all_found}
        print(f"결론: 응답한 조합 {all_found}")
        print(f"  base.yaml 의 baudrate 를 {sorted(bauds)} 중 하나로,")
        print("  left_address/right_address 를 위 address 값으로 맞출 것.")


if __name__ == "__main__":
    main()
