"""Shared protocol helpers for serial communication tests."""

import struct


class OP:
    """Operation codes matching Arduino firmware."""
    MOVE = 1
    ROTATE = 2
    STOP = 3


def build_command_packet(seq_id, operation, arg):
    """
    Build a 4-byte command packet.

    Args:
        seq_id: Sequence ID (0-255)
        operation: Operation code (OP.MOVE, OP.ROTATE, OP.STOP)
        arg: Argument (0-255)

    Returns:
        bytes: (seq_id, operation, arg, checksum)
        Checksum is the additive sum of the first 3 bytes, truncated to uint8_t.
    """
    checksum = (seq_id + operation + arg) & 0xFF
    return struct.pack('<BBBB', seq_id, operation, arg, checksum)


def parse_response_packet(data):
    """
    Parse a 2-byte response packet.

    Args:
        data: bytes to parse

    Returns:
        tuple: (seq_id, checksum) or None if data is malformed
    """
    if len(data) < 2:
        return None
    seq_id, checksum = struct.unpack('<BB', data[:2])
    return (seq_id, checksum)
