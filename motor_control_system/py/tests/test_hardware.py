"""
Hardware integration tests for serial protocol.

Tests the Arduino firmware by sending command packets and verifying responses.

Usage:
    pytest py/tests --port /dev/cu.usbserial-XXXX
    pytest py/tests --port COM3 --baud 9600
"""

import time

import pytest
from protocol import (OP, RESPONSE_SIZE, Status, build_command_packet,
                      parse_response_packet)

# From config.hpp. MAX_COMMAND_MS is an emergency stop only, not the normal
# path - a real distance/angle command should always end via progress
# (reaching its target) or STALL_TIMEOUT_MS (no odometer progress) long
# before it. Serial read timeouts need margin above whichever applies.
MAX_COMMAND_S = 120
STALL_TIMEOUT_S = 3
READ_MARGIN_S = 5


def read_response(conn, timeout=2):
    """Read and parse the next response packet without sending anything -
    for a queued op's completion status, which the board sends
    asynchronously once the command finishes."""
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
    """Send one command with human-readable args and return its immediate
    response - e.g. call(conn, OP.QUEUE_MOVE_DIST, distance=1.0, direction=1)
    instead of hand-building a packet. For queued ops this immediate
    response is just the "queued" ack; use read_response() afterward for
    the completion status once the command finishes."""
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
    """Test: send MOVE_DIST/ROTATE_ANGLE command and verify it's queued then
    completed via odometer feedback. Requires WHEEL_RADIUS_M/GEAR_RATIO/
    TRACK_WIDTH_M in config.hpp to be calibrated for the real robot, or this
    will run until MAX_COMMAND_MS and report STATUS_TIMEOUT instead."""
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
    # TODO: Motor A - motor 1's encoder is disabled, uncomment once re-enabled
    # assert (resp.w1 > 0) == (direction > 0)
    # assert (resp.v1 > 0) == (direction > 0)
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
    """Test: DEBUG_RUN_N_ROTATIONS is queued then completes (STATUS_SUCCESS,
    or STATUS_TIMEOUT if ENCODER_PULSES_PER_REV isn't calibrated yet) without
    hanging - it measures raw shaft rotations directly, independent of
    WHEEL_RADIUS_M/GEAR_RATIO/TRACK_WIDTH_M."""
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
    resp = call(serial_conn, OP.RESET_DISTANCE, seq_id=60)
    assert resp.seq_id == 60
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=61, duration=0.3, direction=1)
    assert resp.seq_id == 61
    assert resp.status == Status.OK

    resp = read_response(serial_conn, timeout=5)
    assert resp.seq_id == 61
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.POLL_DISTANCE, seq_id=62)
    assert resp.seq_id == 62
    assert resp.status == Status.SUCCESS
    assert resp.d2 > 0


@pytest.mark.parametrize("direction", [1, -1])
def test_move_dist_matches_polled_distance(serial_conn, direction):
    """Test: after resetting the odometer and commanding a known 1m
    MOVE_DIST, OP_POLL_DISTANCE reports a cumulative distance close to what
    was actually commanded (correctly signed) - not the ~0.5m the old
    dsA/dsB averaging bug would have given, and not ~0."""
    resp = call(serial_conn, OP.RESET_DISTANCE, seq_id=80)
    assert resp.status == Status.SUCCESS

    resp = call(
        serial_conn, OP.QUEUE_MOVE_DIST, seq_id=81, distance=1.0, direction=direction
    )
    assert resp.status == Status.OK

    resp = read_response(serial_conn, timeout=MAX_COMMAND_S + READ_MARGIN_S)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.POLL_DISTANCE, seq_id=82)
    assert resp.status == Status.SUCCESS
    # Generous tolerance: real hardware has calibration noise and a bit of
    # tick-discretized ramp-down overshoot - this only needs to rule out
    # "roughly right" vs. "way off" (the averaging bug, or nothing moved).
    assert direction * resp.d2 == pytest.approx(1.0, abs=0.15)


def test_stop_interrupts_active_move(serial_conn):
    """Test: OP_STOP mid-move halts immediately. The interrupted move's own
    completion response never arrives - OP_STOP clears isActive directly,
    bypassing the control tick's normal done-handling that would otherwise
    send it - so only STOP's own response should follow. Doesn't assert on
    velocity settling to ~0: motorDriver.stop() cuts power but doesn't brake,
    so real mechanical coast-down time is a hardware property, not something
    the firmware guarantees - is_active is the actual contract here."""
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
    assert not resp.is_active


def test_direction_reversal_nets_out(serial_conn):
    """Test: moving forward then backward for the same duration nets the
    signed cumulative distance back close to zero - exercises
    pollAndAccumulate's per-command sign, not just a single command's."""
    resp = call(serial_conn, OP.RESET_DISTANCE, seq_id=90)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=91, duration=0.3, direction=1)
    assert resp.status == Status.OK
    resp = read_response(serial_conn, timeout=5)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.QUEUE_MOVE, seq_id=92, duration=0.3, direction=-1)
    assert resp.status == Status.OK
    resp = read_response(serial_conn, timeout=5)
    assert resp.status == Status.SUCCESS

    resp = call(serial_conn, OP.POLL_DISTANCE, seq_id=93)
    assert resp.status == Status.SUCCESS
    assert resp.d2 == pytest.approx(0.0, abs=0.1)


def test_move_dist_stalls_when_not_progressing(serial_conn):
    """Test: MOVE_DIST with direction=0 never actually turns the wheel -
    motorDriver.stop() short-circuits regardless of the computed PWM (see
    applyDirection() in motor_driver.hpp) - so accumulatedM never advances.
    Should abort via the fast stall detector (STALL_TIMEOUT_MS), not the
    much larger MAX_COMMAND_MS emergency-stop backstop."""
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
