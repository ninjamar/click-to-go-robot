import struct
from enum import IntEnum


class OP:
    MOVE = 1
    ROTATE = 2
    STOP = 3


class Status(IntEnum):
    OK = 0
    SUCCESS = 1
    REJECTED_FULL = 2


def build_command_packet(seq_id, operation, arg, direction=0):
    return struct.pack("<BBQb", seq_id, operation, arg, direction)


def parse_response_packet(data):
    seq_id, status = struct.unpack("<BB", data[:2])
    return seq_id, Status(status)
