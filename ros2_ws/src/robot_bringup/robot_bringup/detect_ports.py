"""Identify which /dev/ttyUSB* device is the lidar vs. the motor-control Nano.

USB enumeration order isn't guaranteed across replugs, so ttyUSB0/ttyUSB1 can point
at either device depending on boot-time enumeration order. The lidar streams scan
data continuously and unprompted; the Nano firmware stays silent until polled. That
behavioral difference is used to identify each port instead of trusting a fixed path.
"""

import glob

import serial

LIDAR_BAUD = 230400
PROBE_SECONDS = 1.0
LIDAR_MIN_BYTES = 32  # a genuine lidar stream produces far more than this in 1s


def _looks_like_lidar(port: str) -> bool:
    with serial.Serial(port, LIDAR_BAUD, timeout=PROBE_SECONDS) as conn:
        return len(conn.read(256)) >= LIDAR_MIN_BYTES


def detect_serial_ports(candidates: list[str] | None = None) -> tuple[str, str]:
    """Return (nano_port, lidar_port), identified by behavior rather than path.

    Raises RuntimeError if detection isn't unambiguous.
    """
    if candidates is None:
        candidates = sorted(glob.glob("/dev/ttyUSB*"))

    if len(candidates) != 2:
        raise RuntimeError(
            f"Expected exactly 2 serial devices (lidar + Nano), found {candidates}"
        )

    streaming = [port for port in candidates if _looks_like_lidar(port)]
    if len(streaming) != 1:
        raise RuntimeError(
            f"Couldn't uniquely identify the lidar port among {candidates} "
            f"(streaming: {streaming}) — check connections"
        )

    lidar_port = streaming[0]
    nano_port = next(port for port in candidates if port != lidar_port)
    return nano_port, lidar_port
