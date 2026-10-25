"""Find a /dev/ttyUSB* by the kernel driver bound to it.

A USB device that drops and reconnects comes back under a new ttyUSB number, and inside
the container its /dev node may not exist yet. The kernel driver (read from sysfs) says
which device is which, without touching the port.
"""

import os
import stat
from pathlib import Path

SYS_TTY = Path("/sys/class/tty")
LIDAR_DRIVER = "cp210x"  # the lidar's CP2102
NANO_DRIVER = "ch341-uart"  # the Nano clone's CH340


def _usb_serial_drivers() -> dict[str, str]:
    """Map each /dev/ttyUSB* the kernel knows about to its driver name, via sysfs."""
    ports = {}
    for tty in sorted(SYS_TTY.glob("ttyUSB*")):
        driver = tty / "device" / "driver"
        if driver.exists():
            ports[f"/dev/{tty.name}"] = driver.resolve().name
    return ports


def _ensure_device_node(port: str) -> None:
    """Create the /dev node if the container lacks it (a reconnected device gets a new one)."""
    if os.path.exists(port):
        return
    major, minor = (SYS_TTY / Path(port).name / "dev").read_text().strip().split(":")
    os.mknod(port, 0o660 | stat.S_IFCHR, os.makedev(int(major), int(minor)))


def find_port(driver: str) -> str:
    """Return the /dev path of the one tty bound to `driver`, creating its node if needed."""
    matches = [p for p, d in _usb_serial_drivers().items() if d == driver]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one {driver} serial device, found {matches}")
    _ensure_device_node(matches[0])
    return matches[0]
