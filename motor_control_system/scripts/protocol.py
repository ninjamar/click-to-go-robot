import struct
from enum import IntEnum


class OP:
    QUEUE_MOVE = 1
    QUEUE_ROTATE = 2
    STOP = 3
    POLL_W = 4
    QUEUE_MOVE_DIST = 5
    QUEUE_ROTATE_ANGLE = 6
    DEBUG_RUN_N_ROTATIONS = 7
    DEBUG_SET_PWM = 8


class Status(IntEnum):
    OK = 0
    SUCCESS = 1
    REJECTED_FULL = 2
    TIMEOUT = 3


def build_command_packet(seq_id, operation, arg, direction=0):
    return struct.pack("<BBfb", seq_id, operation, arg, direction)


RESPONSE_FORMAT = "<BBffff"
RESPONSE_SIZE = struct.calcsize(RESPONSE_FORMAT)  # 18


def parse_response_packet(data):
    seq_id, status, w1, w2, v1, v2 = struct.unpack(
        RESPONSE_FORMAT, data[:RESPONSE_SIZE]
    )
    return seq_id, Status(status), w1, w2, v1, v2
