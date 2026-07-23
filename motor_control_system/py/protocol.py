# AUTO-GENERATED FILE - DO NOT EDIT BY HAND.
# Run `just generate-protocol` to regenerate from serial_protocol.hpp.

# ruff: noqa
# isort: skip_file
# fmt: off

import struct
from collections import namedtuple
from enum import IntEnum


class OP(IntEnum):
    QUEUE_MOVE = 1
    QUEUE_ROTATE = 2
    STOP = 3
    POLL_W = 4
    QUEUE_MOVE_DIST = 5
    QUEUE_ROTATE_ANGLE = 6
    DEBUG_RUN_N_ROTATIONS = 7
    DEBUG_SET_PWM = 8
    RESET_DISTANCE = 9
    POLL_DISTANCE = 10


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
    "reset_distance": OP.RESET_DISTANCE,
    "poll_distance": OP.POLL_DISTANCE,
}


def build_command_packet(seq_id, operation, arg, direction):
    return struct.pack("<BBfb", seq_id, operation, arg, direction)


RESPONSE_FORMAT = "<BBBffffff"
RESPONSE_SIZE = struct.calcsize(RESPONSE_FORMAT)

ResponsePacket = namedtuple("ResponsePacket", ["seq_id", "status", "is_active", "w1", "w2", "v1", "v2", "d1", "d2"])


def parse_response_packet(data):
    seq_id, status, is_active, w1, w2, v1, v2, d1, d2 = struct.unpack(
        RESPONSE_FORMAT, data[:RESPONSE_SIZE]
    )
    return ResponsePacket(seq_id, Status(status), is_active, w1, w2, v1, v2, d1, d2)

# fmt: on
