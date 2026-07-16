#!/ usr / bin / env python
"""
Send a single command packet to the board via serial.

Usage:
    python send_command.py move 50 --port /dev/cu.usbserial-XXXX
    python send_command.py 1 50 --port /dev/cu.usbserial-XXXX -- baud 9600
    python send_command.py rotate 90 --port /dev/cu.usbserial-XXXX --seq-id 42
"""

import argparse
import time

import serial
from protocol import OP, Status, build_command_packet, parse_response_packet


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

    conn = serial.Serial(args.port, args.baud, timeout=2)
    time.sleep(2)
    conn.reset_input_buffer()

    cmd = build_command_packet(args.seq_id, operation, args.arg)
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
