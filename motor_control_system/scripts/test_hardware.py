"""
Hardware integration tests for serial protocol.

Tests the Arduino firmware by sending command packets and verifying responses.

Usage:
    pytest scripts/ --port /dev/cu.usbserial-XXXX
    pytest scripts/ --port COM3 --baud 9600
"""

import pytest
from protocol import OP, Status, build_command_packet, parse_response_packet


@pytest.mark.parametrize(
    "seq_id,op,param,direction",
    [
        (1, OP.MOVE, 50, 1),
        (2, OP.ROTATE, 90, 0),
    ],
)
def test_move_rotate_command(serial_conn, seq_id, op, param, direction):
    """Test: send MOVE/ROTATE command and verify it's queued then completed."""
    cmd = build_command_packet(seq_id, op, param, direction)
    serial_conn.write(cmd)

    resp_data = serial_conn.read(2)
    resp_seq_id, resp_status = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id
    assert resp_status == Status.OK

    resp_data = serial_conn.read(2)
    resp_seq_id, resp_status = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id
    assert resp_status == Status.SUCCESS


def test_stop_command(serial_conn):
    """Test: send STOP command and verify it completes immediately."""
    cmd = build_command_packet(3, OP.STOP, 0, 0)
    serial_conn.write(cmd)

    resp_data = serial_conn.read(2)
    resp_seq_id, resp_status = parse_response_packet(resp_data)
    assert resp_seq_id == 3
    assert resp_status == Status.SUCCESS


def test_queue_full_rejection(serial_conn):
    """Test: send 5 commands rapidly and verify the 5th is rejected due to queue full."""
    for seq_id in [10, 11, 12, 13, 14]:
        cmd = build_command_packet(seq_id, OP.MOVE, 10000, 0)
        serial_conn.write(cmd)

    responses = []
    for _ in range(5):
        resp_data = serial_conn.read(2)
        resp_seq_id, resp_status = parse_response_packet(resp_data)
        responses.append(resp_status)

    assert all(s == Status.OK for s in responses[:4])
    assert responses[4] == Status.REJECTED_FULL
