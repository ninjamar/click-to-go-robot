import struct


class OP:
    MOVE = 1
    ROTATE = 2
    STOP = 3


def build_command_packet(seq_id, operation, arg):
    return struct.pack("<BBQ", seq_id, operation, arg)


def parse_response_packet(data):
    seq_id = struct.unpack("<B", data[:1])
    return seq_id
