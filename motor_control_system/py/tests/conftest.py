"""pytest configuration and fixtures for hardware integration tests."""

import pytest
import serial
from protocol import OP, SerialClient

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
def client(request):
    """Opens the serial client once for all tests; waits for the DTR board reset first."""
    port_name = request.config.getoption("--port")
    baud = request.config.getoption("--baud")

    try:
        client = SerialClient(port_name, baud)
    except serial.SerialException as e:
        pytest.fail(f"Failed to open serial port {port_name}: {e}")

    yield client

    client.close()


@pytest.fixture(autouse=True)
def reset_board(client):
    """Stop the motor and drain stray bytes so a leftover response can't desync the next test."""
    client.send(OP.STOP, seq_id=0)
    client.read_response(timeout=STRAY_RESPONSE_TIMEOUT_S)
    client.drain()

    yield

    client.send(OP.STOP, seq_id=0)
    client.read_response(timeout=STRAY_RESPONSE_TIMEOUT_S)
    client.drain()
