#!/bin/bash
set -euo pipefail

# CANable2 slcan setup. No motor commands are sent.
CHECK_ONLY=false
NON_INTERACTIVE=false
INTERFACE=can0
DEVICE=""
while (( $# )); do
    case "$1" in
        --check) CHECK_ONLY=true; shift ;;
        --non-interactive) NON_INTERACTIVE=true; shift ;;
        --interface)
            [[ $# -ge 2 ]] || { echo 'ERROR: --interface needs a name'; exit 1; }
            INTERFACE="$2"; shift 2 ;;
        --*) echo "ERROR: Unknown option: $1"; exit 1 ;;
        *) [[ -z "$DEVICE" ]] || { echo 'ERROR: Only one serial device is allowed'; exit 1; }
           DEVICE="$1"; shift ;;
    esac
done
[[ "$INTERFACE" =~ ^[a-zA-Z0-9_][a-zA-Z0-9_.-]{0,14}$ ]] || { echo 'ERROR: Invalid interface name'; exit 1; }

is_up() {
    ip -j link show dev "$INTERFACE" 2>/dev/null | python3 -c \
        'import json,sys; d=json.load(sys.stdin); sys.exit(not(d and "UP" in d[0]["flags"]))' 2>/dev/null
}

# Existing UP interfaces need neither USB discovery nor administrator privileges.
if ! "$CHECK_ONLY" && is_up; then
    echo "$INTERFACE already UP; using existing interface (device/bitrate not reconfigured)."
    exit 0
fi

PRIV=()
if (( EUID != 0 )); then
    PRIV=(sudo)
    if "$NON_INTERACTIVE"; then PRIV=(sudo -n); fi
fi

if ! "$CHECK_ONLY" && ip link show dev "$INTERFACE" &>/dev/null; then
    "${PRIV[@]}" ip link set dev "$INTERFACE" up
else
    # Stable USB identity: ttyACM0/ttyACM1 can change after reconnecting.
    if [[ -z "$DEVICE" ]]; then
        shopt -s nullglob nocaseglob
        candidates=(/dev/serial/by-id/*canable*)
        if (( ${#candidates[@]} != 1 )); then
            echo "ERROR: Expected one CANable adapter, found ${#candidates[@]}."
            echo 'Specify the serial device explicitly (launch: can_device:=/dev/serial/by-id/...).'
            exit 1
        fi
        DEVICE="${candidates[0]}"
    fi
    [[ -c "$DEVICE" ]] || { echo "ERROR: CAN serial device not found: $DEVICE"; exit 1; }
    echo "CAN adapter: $DEVICE -> $(readlink -f -- "$DEVICE")"
    if "$CHECK_ONLY"; then
        echo 'Device detection OK. No interface changes or motor commands sent.'
        exit 0
    fi
    "${PRIV[@]}" modprobe can
    "${PRIV[@]}" modprobe can_raw
    "${PRIV[@]}" modprobe slcan
    "${PRIV[@]}" slcand -o -c -s8 "$DEVICE" "$INTERFACE"
    for attempt in {1..20}; do
        if ip link show dev "$INTERFACE" &>/dev/null; then break; fi
        sleep 0.1
    done
    "${PRIV[@]}" ip link set dev "$INTERFACE" up
fi
if ! is_up; then
    echo "ERROR: $INTERFACE did not reach administrative UP state."
    exit 1
fi
ip -br link show dev "$INTERFACE"
