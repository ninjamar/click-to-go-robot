import struct
from enum import IntEnum


class OP:
    QUEUE_MOVE = 1
    QUEUE_ROTATE = 2
    STOP = 3
    POLL_W = 4


class Status(IntEnum):
    OK = 0
    SUCCESS = 1
    REJECTED_FULL = 2


def build_command_packet(seq_id, operation, arg, direction=0):
    return struct.pack("<BBQb", seq_id, operation, arg, direction)


def parse_response_packet(data):
    fmt = "<BBff"
    size = struct.calcsize(fmt)  # 10

    seq_id, status, w1, w2 = struct.unpack(fmt, data[:size])
    return seq_id, Status(status), w1, w2
