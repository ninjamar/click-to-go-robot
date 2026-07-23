"""pytest configuration and fixtures for hardware integration tests."""

import time

import pytest
import serial
from protocol import OP, RESPONSE_SIZE, build_command_packet


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
    """
    Session-scoped fixture that opens the serial port once for all tests.

    Waits for board reset (DTR) and clears the input buffer before yielding.
    Closes the port after all tests complete.
    """
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


def _drain(conn, timeout=0.05):
    """Discard any buffered response bytes for up to `timeout` seconds."""
    conn.timeout = timeout
    while conn.read(RESPONSE_SIZE):
        pass


@pytest.fixture(autouse=True)
def reset_board(serial_conn):
    """
    Stop the motor and drain stray bytes before and after each test.

    serial_conn is session-scoped and shared across all tests, so without
    this a late or leftover response from one test (e.g. a timed-out command)
    can desync the next test's reads.
    """
    serial_conn.write(build_command_packet(0, OP.STOP, 0, 0))
    serial_conn.timeout = 2
    serial_conn.read(RESPONSE_SIZE)
    _drain(serial_conn)
    serial_conn.timeout = 2

    yield

    serial_conn.write(build_command_packet(0, OP.STOP, 0, 0))
    serial_conn.timeout = 2
    serial_conn.read(RESPONSE_SIZE)
    _drain(serial_conn)
    serial_conn.timeout = 2
