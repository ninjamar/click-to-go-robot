"""
Hardware integration tests for serial protocol.

Tests the Arduino firmware by sending command packets and verifying responses.

Usage:
    pytest scripts/ --port /dev/cu.usbserial-XXXX
    pytest scripts/ --port COM3 --baud 9600
"""

import time

import pytest
from protocol import (OP, RESPONSE_SIZE, Status, build_command_packet,
                      parse_response_packet)


@pytest.mark.parametrize(
    "seq_id,op,param,direction",
    [
        (1, OP.QUEUE_MOVE, 0.05, 1),
        (2, OP.QUEUE_ROTATE, 0.05, 0),
    ],
)
def test_move_rotate_command(serial_conn, seq_id, op, param, direction):
    """Test: send MOVE/ROTATE command and verify it's queued then completed."""
    cmd = build_command_packet(seq_id, op, param, direction)
    serial_conn.write(cmd)

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id
    assert resp_status == Status.OK

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id
    assert resp_status == Status.SUCCESS


@pytest.mark.parametrize(
    "seq_id,op,param,direction",
    [
        (5, OP.QUEUE_MOVE_DIST, 0.05, 1),
        (6, OP.QUEUE_ROTATE_ANGLE, 0.1, 1),
    ],
)
def test_move_dist_rotate_angle_command(serial_conn, seq_id, op, param, direction):
    """Test: send MOVE_DIST/ROTATE_ANGLE command and verify it's queued then
    completed via odometer feedback. Requires WHEEL_RADIUS_M/GEAR_RATIO/
    TRACK_WIDTH_M in config.hpp to be calibrated for the real robot, or this
    will run until MAX_COMMAND_MS and report STATUS_TIMEOUT instead."""
    cmd = build_command_packet(seq_id, op, param, direction)
    serial_conn.write(cmd)

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id
    assert resp_status == Status.OK

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == seq_id
    assert resp_status == Status.SUCCESS


@pytest.mark.parametrize(
    "move_seq_id,poll_seq_id,direction",
    [
        (30, 31, 1),
        (32, 33, -1),
    ],
)
def test_poll_w_while_moving(serial_conn, move_seq_id, poll_seq_id, direction):
    """Test: motor B spinning under OP_QUEUE_MOVE reports nonzero, correctly
    signed angular/linear velocity from OP_POLL_W."""
    move_cmd = build_command_packet(move_seq_id, OP.QUEUE_MOVE, 1.0, direction)
    serial_conn.write(move_cmd)

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == move_seq_id
    assert resp_status == Status.OK

    time.sleep(0.3)  # let motor ramp up so it's actually spinning

    poll_cmd = build_command_packet(poll_seq_id, OP.POLL_W, 0, 0)
    serial_conn.write(poll_cmd)

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, w1, w2, v1, v2 = parse_response_packet(resp_data)
    assert resp_seq_id == poll_seq_id
    assert resp_status == Status.SUCCESS
    # TODO: MOTOR_A - motor 1's encoder is disabled, uncomment once re-enabled
    # assert (w1 > 0) == (direction > 0)
    # assert (v1 > 0) == (direction > 0)
    assert (w2 > 0) == (direction > 0)
    assert (v2 > 0) == (direction > 0)

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == move_seq_id
    assert resp_status == Status.SUCCESS


def test_stop_command(serial_conn):
    """Test: send STOP command and verify it completes immediately."""
    cmd = build_command_packet(3, OP.STOP, 0, 0)
    serial_conn.write(cmd)

    resp_data = serial_conn.read(RESPONSE_SIZE)
    resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
    assert resp_seq_id == 3
    assert resp_status == Status.SUCCESS


def test_queue_full_rejection(serial_conn):
    """Test: send 5 commands rapidly and verify the 5th is rejected due to queue full."""
    for seq_id in [10, 11, 12, 13, 14]:
        cmd = build_command_packet(seq_id, OP.QUEUE_MOVE, 10.0, 0)
        serial_conn.write(cmd)

    responses = []
    for _ in range(5):
        resp_data = serial_conn.read(RESPONSE_SIZE)
        resp_seq_id, resp_status, _, _, _, _ = parse_response_packet(resp_data)
        responses.append(resp_status)

    assert all(s == Status.OK for s in responses[:4])
    assert responses[4] == Status.REJECTED_FULL
