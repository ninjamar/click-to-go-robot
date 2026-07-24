"""pytest configuration and fixtures for hardware integration tests."""

import time

import pytest
import serial
from protocol import OP, RESPONSE_SIZE, build_command_packet

# From config.hpp: a queued command can go this long with no odometer
# progress before the firmware force-completes it on its own (STALL_TIMEOUT_MS),
# even after we've already sent STOP. reset_board's post-STOP read must wait at
# least this long, or a stray completion for the *previous* test's command can
# still land after we've decided the buffer is clean and desync the next test.
STALL_TIMEOUT_S = 3
STRAY_RESPONSE_TIMEOUT_S = STALL_TIMEOUT_S + 2


def pytest_addoption(parser):
    """Add --port and --baud options to pytest CLI."""
    parser.addoption(
        "--port",
        action="store",
        required=True,
        help="Serial port (e.g. /dev/cu.usbserial-XXXX or COM3)",
    )
    parser.addoption(
        "--baud",
        action="store",
        type=int,
        default=57600,
        help="Baud rate (default 57600)",
    )


@pytest.fixture(scope="session")
def serial_conn(request):
    """Opens the serial port once for all tests; waits for the DTR board reset first."""
    port_name = request.config.getoption("--port")
    baud = request.config.getoption("--baud")

    try:
        conn = serial.Serial(port_name, baud, timeout=2)
    except serial.SerialException as e:
        pytest.fail(f"Failed to open serial port {port_name}: {e}")

    # Wait for board to finish reset(opening serial triggers DTR reset on Nano)
    time.sleep(2)

    # Clear any stale data in the buffer
    conn.reset_input_buffer()

    yield conn

    conn.close()


def _drain(conn, timeout=0.2):
    """Discard any buffered response bytes for up to `timeout` seconds."""
    conn.timeout = timeout
    while conn.read(RESPONSE_SIZE):
        pass


@pytest.fixture(autouse=True)
def reset_board(serial_conn):
    """Stop the motor and drain stray bytes so a leftover response can't desync the next test."""
    serial_conn.write(build_command_packet(0, OP.STOP, 0, 0))
    serial_conn.timeout = STRAY_RESPONSE_TIMEOUT_S
    serial_conn.read(RESPONSE_SIZE)
    _drain(serial_conn)
    serial_conn.timeout = 2

    yield

    serial_conn.write(build_command_packet(0, OP.STOP, 0, 0))
    serial_conn.timeout = STRAY_RESPONSE_TIMEOUT_S
    serial_conn.read(RESPONSE_SIZE)
    _drain(serial_conn)
    serial_conn.timeout = 2
