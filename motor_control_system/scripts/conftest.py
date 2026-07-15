"""pytest configuration and fixtures for hardware integration tests."""

import time

import pytest
import serial


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

    # Wait for board to finish reset (opening serial triggers DTR reset on Nano)
    time.sleep(2)

    # Clear any stale data in the buffer
    conn.reset_input_buffer()

    yield conn

    conn.close()
