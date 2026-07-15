#!/usr/bin/env python
"""Send a single command packet to the board via serial.

Usage:
    python send_command.py move 50 --port /dev/cu.usbserial-XXXX
    python send_command.py 1 50 --port /dev/cu.usbserial-XXXX --baud 9600
    python send_command.py rotate 90 --port /dev/cu.usbserial-XXXX --seq-id 42
"""

import argparse
import sys
import time

import serial

from protocol import OP, build_command_packet, parse_response_packet


def parse_operation(op_str):
    op_map = {"move": OP.MOVE, "rotate": OP.ROTATE, "stop": OP.STOP}
    return op_map[op_str.lower()]

def main():
    parser = argparse.ArgumentParser(description="Send a command packet to the board.")
    parser.add_argument(
        "operation",
        help="Operation: 'move', 'rotate', 'stop'",
    )
    parser.add_argument(
        "arg",
        type=int,
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

    args = parser.parse_args()

    operation = parse_operation(args.operation)
    
    # Open serial connection
    try:
        conn = serial.Serial(args.port, args.baud, timeout=2)
    except serial.SerialException as e:
        print(f"Error: Failed to open serial port {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        # Wait for board reset (opening serial triggers DTR reset on some boards)
        time.sleep(2)

        # Clear any stale data
        conn.reset_input_buffer()

        # Build and send command
        cmd = build_command_packet(args.seq_id, operation, args.arg)
        print(f"Sending: {cmd.hex()}")
        conn.write(cmd)

        # Read response
        resp_data = conn.read(2)

        if not resp_data:
            print("Error: No response received from board", file=sys.stderr)
            sys.exit(1)

        print(f"Received: {resp_data.hex()}")

        # Parse response
        resp = parse_response_packet(resp_data)
        if resp is None:
            print(f"Error: Failed to parse response", file=sys.stderr)
            sys.exit(1)

        resp_seq_id, resp_checksum = resp
        print(f"Response: seq_id={resp_seq_id}, checksum={resp_checksum}")

        # Verify seq_id matches (board echoes back the seq_id)
        if resp_seq_id == args.seq_id:
            print("Seq ID matches")
        else:
            print(f"Seq ID mismatch: sent {args.seq_id}, got {resp_seq_id}", file=sys.stderr)
            sys.exit(1)

    finally:
        conn.close()


if __name__ == "__main__":
    main()
