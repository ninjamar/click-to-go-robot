#!/usr/bin/env python
"""
Send a chain of command packets to the board over a single serial connection.

Usage (run from motor_control_system/py/):
    python -m scripts.send_command "move_dist:1,poll_dist" --port /dev/cu.usbserial-XXXX
"""

import argparse
import time

import serial
from protocol import (CLI_NAMES, OP, QUEUED_OPS, RESPONSE_SIZE, Status,
                      build_command_packet, parse_response_packet)


def parse_chain(value):
    ops = []
    for item in value.split(","):
        name, _, arg_str = item.strip().partition(":")
        if name not in CLI_NAMES:
            raise argparse.ArgumentTypeError(f"Unknown operation: {name}")
        ops.append((CLI_NAMES[name], float(arg_str) if arg_str else 0.0))
    return ops


def read_response(conn):
    resp_data = b""
    while len(resp_data) < RESPONSE_SIZE:
        chunk = conn.read(RESPONSE_SIZE - len(resp_data))
        if not chunk:
            break
        resp_data += chunk
    return resp_data


def main():
    parser = argparse.ArgumentParser(
        description="Send a chain of command packets to the board."
    )
    parser.add_argument(
        "chain",
        type=parse_chain,
        help="Comma-separated op:arg pairs, e.g. 'move_dist:1,poll_dist'. arg is "
        "duration(s) for move/rotate, distance(m) for move_dist, angle(rad) for "
        "rotate_angle, rotations for debug_run_n_rotations, raw PWM (0-255) for "
        "debug_set_pwm, or omitted for stop/poll_w/poll_dist/reset_dist",
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
        help="Starting sequence ID, incremented per op in the chain (default: 1)",
    )
    parser.add_argument(
        "--direction",
        type=int,
        choices=[-1, 0, 1],
        default=1,
        help="Direction applied to every op in the chain: -1 (backward), 0 (stop), 1 (forward) (default: 1)",
    )

    args = parser.parse_args()

    conn = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2)
    conn.reset_input_buffer()

    seq_id = args.seq_id
    for operation, arg in args.chain:
        cmd = build_command_packet(seq_id, operation, arg, args.direction)
        conn.write(cmd)

        conn.timeout = 2
        resp_data = read_response(conn)
        resp_seq_id, resp_status, is_active, w1, w2, v1, v2, d1, d2, x1, x2 = (
            parse_response_packet(resp_data)
        )
        print(f"intake: seq_id={resp_seq_id}, status={resp_status.name}")
        print(
            f"  is_active={bool(is_active)}, w1={w1:.4f} rad/s, w2={w2:.4f} rad/s, "
            f"v1={v1:.4f} m/s, v2={v2:.4f} m/s, d1={d1:.4f} m, d2={d2:.4f} m, "
            f"x1={x1:.4f} m, x2={x2:.4f} m"
        )

        if operation in QUEUED_OPS and resp_status == Status.OK:
            # Distance/angle ops don't know their own duration - wait past MAX_COMMAND_MS in config.hpp.
            if operation in (OP.QUEUE_MOVE, OP.QUEUE_ROTATE):
                conn.timeout = arg + 2
            else:
                conn.timeout = 125.0
            resp_data = read_response(conn)
            resp_seq_id, resp_status, _, w1, w2, v1, v2, _, _, _, _ = (
                parse_response_packet(resp_data)
            )
            print(f"completion: seq_id={resp_seq_id}, status={resp_status.name}")

        seq_id += 1

    conn.close()


if __name__ == "__main__":
    main()
