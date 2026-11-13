
import socket
import struct
import select
import time
import csv
import statistics

# ============================================================
# AK40-10 V2.0 / 4-Motor MIT Torque Feedback Test
# ============================================================

CAN_INTERFACE = "can0"
MOTOR_IDS = [1,2,3,4]

TORQUE_NM = 0.06
FREQUENCY = 100
TEST_DURATION = 100.0

PRINT_INTERVAL = 1
RESPONSE_WINDOW = 0.009

MAX_ABS_SPEED = 30.0
MAX_CONSECUTIVE_MISSES = 5

CSV_FILE = "four_motor_feedback.csv"

# AK40-10 V2.0 MIT parameter ranges
P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -45.5, 45.5
T_MIN, T_MAX = -5.0, 5.0
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

ENABLE = bytes.fromhex("FFFFFFFFFFFFFFFC")
DISABLE = bytes.fromhex("FFFFFFFFFFFFFFFD")


# ============================================================
# Conversion
# ============================================================

def float_to_uint(value, low, high, bits):
    value = max(low, min(high, value))
    return int(
        (value - low) * ((2**bits - 1) / (high - low))
    )


def uint_to_float(value, low, high, bits):
    return low + value * (high - low) / (2**bits - 1)


# ============================================================
# MIT command packing
# ============================================================

def pack_mit(torque):

    p = float_to_uint(0, P_MIN, P_MAX, 16)
    v = float_to_uint(0, V_MIN, V_MAX, 12)
    kp = float_to_uint(0, KP_MIN, KP_MAX, 12)
    kd = float_to_uint(0, KD_MIN, KD_MAX, 12)
    t = float_to_uint(torque, T_MIN, T_MAX, 12)

    return bytes([
        (p >> 8) & 0xFF,
        p & 0xFF,
        (v >> 4) & 0xFF,
        ((v & 0xF) << 4) | ((kp >> 8) & 0xF),
        kp & 0xFF,
        (kd >> 4) & 0xFF,
        ((kd & 0xF) << 4) | ((t >> 8) & 0xF),
        t & 0xFF
    ])


# ============================================================
# CAN transmission
# ============================================================

def send_can(sock, motor_id, payload):

    frame = struct.pack(
        "=IB3x8s",
        motor_id,
        8,
        payload
    )

    sock.send(frame)


def send_all(sock, payload):

    for motor_id in MOTOR_IDS:
        send_can(sock, motor_id, payload)


# ============================================================
# MIT feedback decoding
# ============================================================

def decode_feedback(payload):

    pos_raw = (payload[1] << 8) | payload[2]

    vel_raw = (
        (payload[3] << 4) |
        (payload[4] >> 4)
    )

    tq_raw = (
        ((payload[4] & 0xF) << 8) |
        payload[5]
    )

    return {
        "position": uint_to_float(
            pos_raw, P_MIN, P_MAX, 16
        ),
        "velocity": uint_to_float(
            vel_raw, V_MIN, V_MAX, 12
        ),
        "torque": uint_to_float(
            tq_raw, T_MIN, T_MAX, 12
        ),
        "temperature": payload[6] - 40,
        "error": payload[7]
    }


# ============================================================
# Receive responses from all motors
# ============================================================

def receive_all(sock, deadline):

    responses = {}

    while time.monotonic() < deadline:

        if len(responses) == len(MOTOR_IDS):
            break

        remaining = deadline - time.monotonic()

        readable, _, _ = select.select(
            [sock], [], [], max(0, remaining)
        )

        if not readable:
            break

        raw = sock.recv(16)

        can_id, dlc, payload = struct.unpack(
            "=IB3x8s", raw
        )

        # Ignore CAN error, extended and RTR frames
        if can_id & 0xE0000000:
            continue

        motor_id = can_id & 0x7FF

        if motor_id not in MOTOR_IDS:
            continue

        if dlc != 8:
            continue

        if payload[0] != motor_id:
            continue

        if motor_id in responses:
            continue

        responses[motor_id] = {
            "data": decode_feedback(payload),
            "rx_time": time.monotonic()
        }

    return responses


# ============================================================
# Main
# ============================================================

def main():

    sock = socket.socket(
        socket.PF_CAN,
        socket.SOCK_RAW,
        socket.CAN_RAW
    )

    feedback_log = []
    tx_count = {i: 0 for i in MOTOR_IDS}
    rx_count = {i: 0 for i in MOTOR_IDS}
    rx_times = {i: [] for i in MOTOR_IDS}
    consecutive_misses = {i: 0 for i in MOTOR_IDS}

    start = None

    try:
        sock.bind((CAN_INTERFACE,))

        print("=" * 70)
        print("AK40-10 V2.0 - 4 Motor Torque Test")
        print("=" * 70)
        print("Motor IDs      :", MOTOR_IDS)
        print("Torque command :", TORQUE_NM, "Nm")
        print("Control rate   :", FREQUENCY, "Hz")
        print("Duration       :", TEST_DURATION, "s")
        print("=" * 70)

        neutral = pack_mit(0.0)
        torque_cmd = pack_mit(TORQUE_NM)

        # Start from neutral command
        send_all(sock, neutral)

        # Enable all motors
        send_all(sock, ENABLE)

        time.sleep(0.05)

        # Drain old responses
        while True:
            readable, _, _ = select.select(
                [sock], [], [], 0
            )
            if not readable:
                break
            sock.recv(16)

        period = 1.0 / FREQUENCY
        cycles = int(TEST_DURATION * FREQUENCY)

        start = time.monotonic()

        for cycle in range(cycles):

            target = start + cycle * period

            remaining = target - time.monotonic()

            if remaining > 0:
                time.sleep(remaining)

            # Send 4 torque commands per cycle
            for motor_id in MOTOR_IDS:
                send_can(sock, motor_id, torque_cmd)
                tx_count[motor_id] += 1

            # Collect responses within this cycle
            deadline = target + RESPONSE_WINDOW

            responses = receive_all(
                sock, deadline
            )

            elapsed = time.monotonic() - start

            for motor_id in MOTOR_IDS:

                item = responses.get(motor_id)

                if item is None:
                    consecutive_misses[motor_id] += 1

                    feedback_log.append({
                        "cycle": cycle,
                        "time_s": elapsed,
                        "motor_id": motor_id,
                        "torque_cmd_nm": TORQUE_NM,
                        "rx_ok": 0,
                        "position_rad": "",
                        "velocity_rad_s": "",
                        "torque_fb_nm": "",
                        "temperature_c": "",
                        "error": ""
                    })
                    continue

                consecutive_misses[motor_id] = 0
                rx_count[motor_id] += 1

                data = item["data"]
                rx_time = item["rx_time"]
                rx_times[motor_id].append(rx_time)

                feedback_log.append({
                    "cycle": cycle,
                    "time_s": rx_time - start,
                    "motor_id": motor_id,
                    "torque_cmd_nm": TORQUE_NM,
                    "rx_ok": 1,
                    "position_rad": data["position"],
                    "velocity_rad_s": data["velocity"],
                    "torque_fb_nm": data["torque"],
                    "temperature_c": data["temperature"],
                    "error": data["error"]
                })

                # Stop test on motor error
                if data["error"] != 0:
                    raise RuntimeError(
                        f"Motor {motor_id}: "
                        f"error {data['error']}"
                    )

                # Stop test on excessive speed
                if abs(data["velocity"]) > MAX_ABS_SPEED:
                    raise RuntimeError(
                        f"Motor {motor_id}: "
                        f"speed limit exceeded"
                    )

            # Stop if a motor repeatedly misses feedback
            for motor_id in MOTOR_IDS:
                if (
                    consecutive_misses[motor_id]
                    >= MAX_CONSECUTIVE_MISSES
                ):
                    raise RuntimeError(
                        f"Motor {motor_id}: "
                        "repeated feedback timeout"
                    )

            # Terminal output at 10 Hz
            if cycle % PRINT_INTERVAL == 0:

                print(
                    f"\nTime: {elapsed:.3f} s | "
                    f"CMD: {TORQUE_NM:+.3f} Nm"
                )

                for motor_id in MOTOR_IDS:

                    item = responses.get(motor_id)

                    if item is None:
                        print(
                            f" Motor {motor_id} | RX timeout"
                        )
                        continue

                    data = item["data"]

                    print(
                        f" Motor {motor_id} | "
                        f"FB: {data['torque']:+.3f} Nm | "
                        f"Speed: "
                        f"{data['velocity']:+.3f} rad/s | "
                        f"Temp: "
                        f"{data['temperature']} C | "
                        f"Error: {data['error']}"
                    )

    except KeyboardInterrupt:
        print("\nTest interrupted")

    except Exception as exc:
        print("\nTest stopped:", exc)

    finally:

        # Emergency cleanup
        try:
            neutral = pack_mit(0.0)

            for _ in range(5):
                send_all(sock, neutral)
                time.sleep(0.01)

            send_all(sock, DISABLE)
            print("\nAll motors disabled")

        except Exception as exc:
            print("Disable command failed:", exc)

        finally:
            sock.close()

        # Save results
        if feedback_log:

            with open(
                CSV_FILE, "w", newline=""
            ) as f:

                writer = csv.DictWriter(
                    f,
                    fieldnames=feedback_log[0].keys()
                )

                writer.writeheader()
                writer.writerows(feedback_log)

            print("CSV saved:", CSV_FILE)

        print("\n" + "=" * 70)
        print("FINAL RESULTS")
        print("=" * 70)

        for motor_id in MOTOR_IDS:

            tx = tx_count[motor_id]
            rx = rx_count[motor_id]

            ratio = (
                rx / tx * 100 if tx > 0 else 0
            )

            print(
                f"Motor {motor_id} | "
                f"TX: {tx:3d} | "
                f"RX: {rx:3d} | "
                f"RX/TX: {ratio:5.1f}%"
            )

            times = rx_times[motor_id]

            if len(times) >= 2:

                intervals = [
                    (b - a) * 1000
                    for a, b in zip(
                        times[:-1], times[1:]
                    )
                ]

                print(
                    f"  Mean RX interval: "
                    f"{statistics.mean(intervals):.3f} ms"
                )

                print(
                    f"  Max RX interval : "
                    f"{max(intervals):.3f} ms"
                )


if __name__ == "__main__":
    main()

