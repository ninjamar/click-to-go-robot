#!/usr/bin/env python
"""
Send a single command packet to the board via serial.

Usage (run from motor_control_system/py/):
    python -m scripts.send_command --operation move 2.0 --port /dev/cu.usbserial-XXXX
"""

import argparse
import time

import serial
from protocol import (CLI_NAMES, OP, QUEUED_OPS, RESPONSE_SIZE, Status,
                      build_command_packet, parse_response_packet)


def main():
    parser = argparse.ArgumentParser(description="Send a command packet to the board.")
    parser.add_argument(
        "-op",
        "--operation",
        required=True,
        choices=sorted(CLI_NAMES),
        help="Operation to send",
    )
    parser.add_argument(
        "arg",
        type=float,
        help="Duration (s) for move/rotate, distance (m) for move_dist, "
        "angle (rad) for rotate_angle, rotations for debug_run_n_rotations, "
        "raw PWM (0-255) for debug_set_pwm, or 0 for stop/poll_w",
        default=0.0,
        nargs="?",
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port path",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=57600,
        help="Baud rate",
    )
    parser.add_argument(
        "--seq-id",
        type=int,
        default=1,
        help="Sequence ID (default: 1)",
    )
    parser.add_argument(
        "--direction",
        type=int,
        choices=[-1, 0, 1],
        default=1,
        help="Direction: -1 (backward), 0 (stop), 1 (forward) (default: 1)",
    )

    args = parser.parse_args()
    operation = CLI_NAMES[args.operation]

    conn = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2)
    conn.reset_input_buffer()

    cmd = build_command_packet(args.seq_id, operation, args.arg, args.direction)
    conn.write(cmd)

    resp_data = b""
    while len(resp_data) < RESPONSE_SIZE:
        chunk = conn.read(RESPONSE_SIZE - len(resp_data))
        if not chunk:
            break
        resp_data += chunk
    print(f"DEBUG: received {len(resp_data)} bytes: {resp_data.hex()}")
    resp_seq_id, resp_status, is_active, w1, w2, v1, v2, d1, d2 = parse_response_packet(
        resp_data
    )
    print(f"intake: seq_id={resp_seq_id}, status={resp_status.name}")
    if operation in (OP.POLL_W, OP.POLL_DISTANCE):
        print(
            f"  is_active={bool(is_active)}, w1={w1:.4f} rad/s, w2={w2:.4f} rad/s, "
            f"v1={v1:.4f} m/s, v2={v2:.4f} m/s, d1={d1:.4f} m, d2={d2:.4f} m"
        )

    if operation in QUEUED_OPS and resp_status == Status.OK:
        # Distance/angle ops don't know their own duration - wait past MAX_COMMAND_MS in config.hpp.
        if operation in (OP.QUEUE_MOVE, OP.QUEUE_ROTATE):
            conn.timeout = args.arg + 2
        else:
            conn.timeout = 125.0
        resp_data = b""
        while len(resp_data) < RESPONSE_SIZE:
            chunk = conn.read(RESPONSE_SIZE - len(resp_data))
            if not chunk:
                break
            resp_data += chunk
        print(f"DEBUG: received {len(resp_data)} bytes: {resp_data.hex()}")
        resp_seq_id, resp_status, _, w1, w2, v1, v2, _, _ = parse_response_packet(
            resp_data
        )
        print(f"completion: seq_id={resp_seq_id}, status={resp_status.name}")

    conn.close()


if __name__ == "__main__":
    main()
