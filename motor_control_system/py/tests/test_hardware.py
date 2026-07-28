"""
Hardware integration tests for serial protocol.

Usage: pytest py/tests --port /dev/cu.usbserial-XXXX
"""

import time

import pytest
from protocol import (
    OP,
    RESPONSE_SIZE,
    Status,
    build_command_packet,
    parse_response_packet,
)

# From config.hpp - MAX_COMMAND_MS is an emergency stop only; STALL_TIMEOUT_MS is the normal abort path.
MAX_COMMAND_S = 120
STALL_TIMEOUT_S = 3
READ_MARGIN_S = 5


def read_response(conn, timeout=2):
    """Read+parse the next response without sending anything - for an async queued-op completion."""
    conn.timeout = timeout
    resp_data = conn.read(RESPONSE_SIZE)
    return parse_response_packet(resp_data)


def call(
    conn,
    op,
    *,
    seq_id=1,
    direction=0,
    duration=None,
    distance=None,
    angle=None,
    rotations=None,
    pwm=None,
    timeout=2,
):
    """Send one command with human-readable args; for queued ops this is just the "queued" ack."""
    values = [v for v in (duration, distance, angle, rotations, pwm) if v is not None]
    assert len(values) <= 1, "pass at most one of duration/distance/angle/rotations/pwm"
    arg = values[0] if values else 0.0
    conn.write(build_command_packet(seq_id, op, arg, direction))
    return read_response(conn, timeout=timeout)


@pytest.mark.parametrize(
    "seq_id,op,duration,direction",
    [
        (1, OP.QUEUE_MOVE, 0.05, 1),
        (2, OP.QUEUE_ROTATE, 0.05, 0),
    ],
)
def test_move_rotate_command(serial_conn, seq_id, op, duration, direction):
    """Test: send MOVE/ROTATE command and verify it's queued then completed."""
    resp = call(serial_conn, op, seq_id=seq_id, duration=duration, direction=direction)
    assert resp.seq_id == seq_id
    assert resp.status == Status.OK

    resp = read_response(serial_conn)
    assert resp.seq_id == seq_id
    assert resp.status == Status.SUCCESS


@pytest.mark.parametrize(
    "seq_id,op,kwargs",
    [
        (5, OP.QUEUE_MOVE_DIST, {"distance": 0.05}),
        (6, OP.QUEUE_ROTATE_ANGLE, {"angle": 0.1}),
    ],
)
def test_move_dist_rotate_angle_command(serial_conn, seq_id, op, kwargs):
    """Test: MOVE_DIST/ROTATE_ANGLE is queued then completes via odometer feedback."""
    resp = call(serial_conn, op, seq_id=seq_id, direction=1, **kwargs)
    assert resp.seq_id == seq_id
    assert resp.status == Status.OK

    resp = read_response(serial_conn)
    assert resp.seq_id == seq_id
    assert resp.status == Status.SUCCESS


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
    resp = call(
        serial_conn,
        OP.QUEUE_MOVE,
        seq_id=move_seq_id,
        duration=1.0,
        direction=direction,
    )
    assert resp.seq_id == move_seq_id
    assert resp.status == Status.OK

    time.sleep(0.3)  # let motor ramp up so it's actually spinning

    resp = call(serial_conn, OP.POLL_W, seq_id=poll_seq_id)
    assert resp.seq_id == poll_seq_id
    assert resp.status == Status.SUCCESS
    assert resp.is_active
    assert (resp.w1 > 0) == (direction > 0)
    assert (resp.v1 > 0) == (direction > 0)
    assert (resp.w2 > 0) == (direction > 0)
    assert (resp.v2 > 0) == (direction > 0)

    resp = read_response(serial_conn)
    assert resp.seq_id == move_seq_id
    assert resp.status == Status.SUCCESS


def test_debug_set_pwm(serial_conn):
    """Test: DEBUG_SET_PWM drives at a raw PWM with no ramp, holds until
    STOP, and OP_POLL_W reports plausible signed velocity while it's active."""
    resp = call(serial_conn, OP.DEBUG_SET_PWM, seq_id=40, pwm=150, direction=1)
    assert resp.seq_id == 40
    assert resp.status == Status.OK

    time.sleep(0.3)  # let it spin up (no ramp, but still takes a moment)

    resp = call(serial_conn, OP.POLL_W, seq_id=41)
    assert resp.seq_id == 41
    assert resp.status == Status.SUCCESS
    assert resp.is_active

    assert resp.w1 > 0
    assert resp.v1 > 0
    assert resp.w2 > 0
    assert resp.v2 > 0

    resp = call(serial_conn, OP.STOP, seq_id=42)
    assert resp.seq_id == 42
    assert resp.status == Status.SUCCESS


@pytest.mark.parametrize(
    "seq_id,rotations,direction",
    [
        (50, 1.0, 1),
        (51, 0.5, -1),
    ],
)
def test_debug_run_n_rotations(serial_conn, seq_id, rotations, direction):
    """Test: DEBUG_RUN_N_ROTATIONS completes without hanging (SUCCESS or TIMEOUT)."""
    resp = call(
        serial_conn,
        OP.DEBUG_RUN_N_ROTATIONS,
        seq_id=seq_id,
        rotations=rotations,
        direction=direction,
    )
    assert resp.seq_id == seq_id
    assert resp.status == Status.OK

    resp = read_response(serial_conn, timeout=MAX_COMMAND_S + READ_MARGIN_S)
    assert resp.seq_id == seq_id
    assert resp.status in (Status.SUCCESS, Status.TIMEOUT)


def test_reset_and_poll_distance(serial_conn):
    """Test: RESET_DISTANCE zeroes the persistent odometer, POLL_DISTANCE
    then reports nonzero cumulative distance for wheel B after a move."""
    resp = call(serial_conn, OP.RESET_DIST, seq_id=60)
    assert resp.seq_id == 60
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=61, duration=0.3, direction=1)
    assert resp.seq_id == 61
    assert resp.status == Status.OK

    resp = read_response(serial_conn, timeout=5)
    assert resp.seq_id == 61
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.POLL_DIST, seq_id=62)
    assert resp.seq_id == 62
    assert resp.status == Status.SUCCESS
    assert resp.d2 > 0


@pytest.mark.parametrize("direction", [1, -1])
def test_move_dist_matches_polled_distance(serial_conn, direction):
    """Test: a known 1m MOVE_DIST is reflected (signed) in OP_POLL_DISTANCE afterward."""
    resp = call(serial_conn, OP.RESET_DIST, seq_id=80)
    assert resp.status == Status.SUCCESS

    resp = call(
        serial_conn, OP.QUEUE_MOVE_DIST, seq_id=81, distance=1.0, direction=direction
    )
    assert resp.status == Status.OK

    resp = read_response(serial_conn, timeout=MAX_COMMAND_S + READ_MARGIN_S)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.POLL_DIST, seq_id=82)
    assert resp.status == Status.SUCCESS
    # Generous tolerance for real calibration/overshoot noise - just rules out "way off".
    assert direction * resp.x2 == pytest.approx(1.0, abs=0.15)


def test_stop_interrupts_active_move(serial_conn):
    """Test: OP_STOP mid-move halts immediately; the interrupted move's own completion never arrives."""
    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=100, duration=5.0, direction=1)
    assert resp.status == Status.OK

    time.sleep(0.3)  # let it actually start moving

    resp = call(serial_conn, OP.STOP, seq_id=101)
    assert resp.status == Status.SUCCESS

    # The interrupted move's own completion response should never arrive.
    serial_conn.timeout = 0.5
    assert serial_conn.read(RESPONSE_SIZE) == b""

    resp = call(serial_conn, OP.POLL_W, seq_id=102)
    assert resp.status == Status.SUCCESS
    assert (
        not resp.is_active
    )  # no velocity check: stop() doesn't brake, coast-down isn't guaranteed


def test_direction_reversal_nets_out(serial_conn):
    """Test: forward then backward for the same duration nets x2 back close to zero."""
    resp = call(serial_conn, OP.RESET_DIST, seq_id=90)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=91, duration=0.3, direction=1)
    assert resp.status == Status.OK
    resp = read_response(serial_conn, timeout=5)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=92, duration=0.3, direction=-1)
    assert resp.status == Status.OK
    resp = read_response(serial_conn, timeout=5)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.POLL_DIST, seq_id=93)
    assert resp.status == Status.SUCCESS
    assert resp.x2 == pytest.approx(0.0, abs=0.1)


def test_move_dist_stalls_when_not_progressing(serial_conn):
    """Test: MOVE_DIST with direction=0 never turns the wheel, so it aborts via the fast stall detector."""
    resp = call(serial_conn, OP.QUEUE_MOVE_DIST, seq_id=70, distance=1.0, direction=0)
    assert resp.status == Status.OK

    start = time.time()
    resp = read_response(serial_conn, timeout=STALL_TIMEOUT_S + READ_MARGIN_S)
    elapsed = time.time() - start
    assert resp.status == Status.TIMEOUT
    assert elapsed < MAX_COMMAND_S  # confirms this was the stall path, not the backstop

    resp = call(serial_conn, OP.POLL_W, seq_id=71)
    assert resp.status == Status.SUCCESS
    assert not resp.is_active


def test_stop_command(serial_conn):
    """Test: send STOP command and verify it completes immediately."""
    resp = call(serial_conn, OP.STOP, seq_id=3)
    assert resp.seq_id == 3
    assert resp.status == Status.SUCCESS


def test_queue_full_rejection(serial_conn):
    """Test: send 5 commands rapidly and verify the 5th is rejected due to queue full."""
    for seq_id in [10, 11, 12, 13, 14]:
        serial_conn.write(build_command_packet(seq_id, OP.QUEUE_MOVE, 10.0, 0))

    responses = [read_response(serial_conn) for _ in range(5)]

    assert all(r.status == Status.OK for r in responses[:4])
    assert responses[4].status == Status.REJECTED_FULL
