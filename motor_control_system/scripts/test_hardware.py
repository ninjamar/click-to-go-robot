"""Hardware integration tests for serial protocol.

Tests the Arduino firmware by sending command packets and verifying responses.

Usage:
    pytest scripts/ --port /dev/cu.usbserial-XXXX
    pytest scripts/ --port COM3 --baud 9600
"""

import pytest
from protocol import OP, build_command_packet, parse_response_packet
import struct


@pytest.mark.parametrize("seq_id,op,param", [
    (1, OP.MOVE, 50),
    (2, OP.ROTATE, 90),
    (3, OP.STOP, 0),
])
def test_echo_command(serial_conn, seq_id, op, param):
    """Test: send command and verify it's echoed back with correct seq_id and checksum."""
    cmd = build_command_packet(seq_id, op, param)
    serial_conn.write(cmd)
    resp_data = serial_conn.read(2)

    assert resp_data, f"No response received for {op}"
    resp = parse_response_packet(resp_data)
    assert resp is not None, f"Failed to parse response {resp_data.hex()}"
    assert resp[0] == seq_id, f"Expected seq_id {seq_id}, got {resp[0]}"
    assert resp[1] == seq_id, f"Expected checksum {seq_id}, got {resp[1]}"


def test_invalid_checksum_rejected(serial_conn):
    """Test: send command with bad checksum, expect no response."""
    seq_id = 4
    bad_checksum = 99
    cmd = struct.pack('<BBBB', seq_id, OP.MOVE, 50, bad_checksum)

    serial_conn.write(cmd)
    resp_data = serial_conn.read(2)

    assert not resp_data, f"Expected no response for bad checksum, got {resp_data.hex()}"
