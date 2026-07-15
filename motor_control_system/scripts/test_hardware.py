"""
Hardware integration tests for serial protocol.

Tests the Arduino firmware by sending command packets and verifying responses.

Usage:
    pytest scripts/ --port /dev/cu.usbserial-XXXX
    pytest scripts/ --port COM3 --baud 9600
"""

import pytest
from protocol import OP, build_command_packet, parse_response_packet


@pytest.mark.parametrize(
    "seq_id,op,param",
    [
        (1, OP.MOVE, 50),
        (2, OP.ROTATE, 90),
        (3, OP.STOP, 0),
    ],
)
def test_echo_command(serial_conn, seq_id, op, param):
    """Test: send command and verify it's echoed back with correct seq_id."""
    cmd = build_command_packet(seq_id, op, param)
    serial_conn.write(cmd)
    resp_data = serial_conn.read(1)

    assert resp_data, f"No response received for {op}"
    resp_seq_id = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id, f"Expected seq_id {seq_id}, got {resp_seq_id}"
