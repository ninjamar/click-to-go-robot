# AUTO-GENERATED FILE - DO NOT EDIT BY HAND.
# Run `just generate-protocol` to regenerate from serial_protocol.hpp.

# ruff: noqa
# isort: skip_file
# fmt: off

import struct
from collections import namedtuple
from enum import IntEnum

from fastcrc import crc16

class OP(IntEnum):
    QUEUE_MOVE = 1
    QUEUE_ROTATE = 2
    STOP = 3
    POLL_W = 4
    QUEUE_MOVE_DIST = 5
    QUEUE_ROTATE_ANGLE = 6
    DEBUG_RUN_N_ROTATIONS = 7
    DEBUG_SET_PWM = 8
    RESET_DIST = 9
    POLL_DIST = 10


class Status(IntEnum):
    OK = 0
    SUCCESS = 1
    REJECTED_FULL = 2
    TIMEOUT = 3


QUEUED_OPS = frozenset({OP.QUEUE_MOVE, OP.QUEUE_ROTATE, OP.QUEUE_MOVE_DIST, OP.QUEUE_ROTATE_ANGLE})

CLI_NAMES = {
    "move": OP.QUEUE_MOVE,
    "rotate": OP.QUEUE_ROTATE,
    "stop": OP.STOP,
    "poll_w": OP.POLL_W,
    "move_dist": OP.QUEUE_MOVE_DIST,
    "rotate_angle": OP.QUEUE_ROTATE_ANGLE,
    "debug_run_n_rotations": OP.DEBUG_RUN_N_ROTATIONS,
    "debug_set_pwm": OP.DEBUG_SET_PWM,
    "reset_dist": OP.RESET_DIST,
    "poll_dist": OP.POLL_DIST,
}


def build_command_packet(seq_id, operation, arg, direction):
    payload = struct.pack("<BBfb", seq_id, operation, arg, direction)
    checksum = crc16.modbus(payload)
    return payload + struct.pack("<H", checksum)


RESPONSE_PAYLOAD_SIZE = struct.calcsize("<BBBffffffff")
RESPONSE_FORMAT = "<BBBffffffffH"
RESPONSE_SIZE = struct.calcsize(RESPONSE_FORMAT)

ResponsePacket = namedtuple("ResponsePacket", ["seq_id", "status", "is_active", "w1", "w2", "v1", "v2", "d1", "d2", "x1", "x2"])


def parse_response_packet(data):
    data = data[:RESPONSE_SIZE]
    seq_id, status, is_active, w1, w2, v1, v2, d1, d2, x1, x2, checksum = struct.unpack(
        RESPONSE_FORMAT, data
    )
    if crc16.modbus(data[:RESPONSE_PAYLOAD_SIZE]) != checksum:
        raise ValueError(f"checksum mismatch in response {data!r}")
    return ResponsePacket(seq_id, Status(status), is_active, w1, w2, v1, v2, d1, d2, x1, x2)

# fmt: on
