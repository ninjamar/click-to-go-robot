#!/ usr / bin / env python
"""
Send a single command packet to the board via serial.

Usage:
    python send_command.py --operation move 50 --port /dev/cu.usbserial-XXXX
    python send_command.py --op rotate 90 --port /dev/cu.usbserial-XXXX --seq-id 42
    python send_command.py --operation stop 0 --port /dev/cu.usbserial-XXXX
"""

import argparse
import time

import serial
from protocol import OP, Status, build_command_packet, parse_response_packet


def main():
    parser = argparse.ArgumentParser(description="Send a command packet to the board.")
    parser.add_argument(
        "-op",
        "--operation",
        required=True,
        help="Operation: 'move', 'rotate', 'stop'",
    )
    parser.add_argument(
        "arg",
        type=int,
        help="Duration (ms) for move/rotate, or 0 for stop",
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

    operation = getattr(OP, args.operation.upper())

    conn = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2)
    conn.reset_input_buffer()

    cmd = build_command_packet(args.seq_id, operation, args.arg, args.direction)
    conn.write(cmd)

    resp_data = conn.read(2)
    resp_seq_id, resp_status = parse_response_packet(resp_data)
    print(f"intake: seq_id={resp_seq_id}, status={resp_status.name}")

    if operation != OP.STOP and resp_status == Status.OK:
        conn.timeout = (args.arg / 1000.0) + 2
        resp_data = conn.read(2)
        resp_seq_id, resp_status = parse_response_packet(resp_data)
        print(f"completion: seq_id={resp_seq_id}, status={resp_status.name}")

    conn.close()


if __name__ == "__main__":
    main()
