#!/ usr / bin / env python
"""
Send a single command packet to the board via serial.

Usage:
    python send_command.py --operation move 2.0 --port /dev/cu.usbserial-XXXX
    python send_command.py --op rotate 1.5 --port /dev/cu.usbserial-XXXX --seq-id 42
    python send_command.py --operation move_dist 0.5 --port /dev/cu.usbserial-XXXX
    python send_command.py --operation rotate_angle 1.5708 --port /dev/cu.usbserial-XXXX
    python send_command.py --operation stop 0 --port /dev/cu.usbserial-XXXX
    python send_command.py --operation debug_run_n_rotations 2.5 --port /dev/cu.usbserial-XXXX
    python send_command.py --operation debug_set_pwm 80 --direction 1 --port /dev/cu.usbserial-XXXX
"""

import argparse
import time

import serial
from protocol import (OP, RESPONSE_SIZE, Status, build_command_packet,
                      parse_response_packet)


def main():
    parser = argparse.ArgumentParser(description="Send a command packet to the board.")
    parser.add_argument(
        "-op",
        "--operation",
        required=True,
        help="Operation: 'move', 'rotate', 'move_dist', 'rotate_angle', 'stop', 'poll_w', "
        "'debug_run_n_rotations', 'debug_set_pwm'",
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

    OPERATION_MAP = {
        "move": OP.QUEUE_MOVE,
        "rotate": OP.QUEUE_ROTATE,
        "move_dist": OP.QUEUE_MOVE_DIST,
        "rotate_angle": OP.QUEUE_ROTATE_ANGLE,
        "stop": OP.STOP,
        "poll_w": OP.POLL_W,
        "debug_run_n_rotations": OP.DEBUG_RUN_N_ROTATIONS,
        "debug_set_pwm": OP.DEBUG_SET_PWM,
    }
    operation = OPERATION_MAP[args.operation]

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
    resp_seq_id, resp_status, w1, w2, v1, v2 = parse_response_packet(resp_data)
    print(f"intake: seq_id={resp_seq_id}, status={resp_status.name}")
    if operation == OP.POLL_W:
        print(
            f"  w1={w1:.4f} rad/s, w2={w2:.4f} rad/s, v1={v1:.4f} m/s, v2={v2:.4f} m/s"
        )

    if operation not in (OP.STOP, OP.POLL_W, OP.DEBUG_SET_PWM) and resp_status == Status.OK:
        # Time-based ops know their duration up front (arg is seconds);
        # distance/angle ops don't — the board itself enforces a safety
        # cutoff (MAX_COMMAND_MS in config.hpp), so just wait generously.
        if operation in (OP.QUEUE_MOVE, OP.QUEUE_ROTATE):
            conn.timeout = args.arg + 2
        else:
            conn.timeout = 20.0
        resp_data = b""
        while len(resp_data) < RESPONSE_SIZE:
            chunk = conn.read(RESPONSE_SIZE - len(resp_data))
            if not chunk:
                break
            resp_data += chunk
        print(f"DEBUG: received {len(resp_data)} bytes: {resp_data.hex()}")
        resp_seq_id, resp_status, w1, w2, v1, v2 = parse_response_packet(resp_data)
        print(f"completion: seq_id={resp_seq_id}, status={resp_status.name}")

    conn.close()


if __name__ == "__main__":
    main()
