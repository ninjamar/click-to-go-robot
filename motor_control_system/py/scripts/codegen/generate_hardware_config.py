"""
Generate hardware.yaml from config.hpp via a real libclang parse.

Extracts BAUD_RATE and TRACK_WIDTH_M from the firmware and renders them into
ros2_ws/src/robot_bringup/config/hardware.yaml alongside the non-firmware constants
(serial_port, emergency_stop_distance, scan_min_range) that are hardcoded here.

Usage: just generate-config (requires compile_commands.json - run `pio run -t compiledb` first)
"""

import os
from pathlib import Path

import clang.cindex

from .firmware_parse import collect_decls, parse_firmware

PROJECT_DIR = Path(__file__).parents[3]  # motor_control_system/
CONFIG_HPP = PROJECT_DIR / "src/config.hpp"

REPO_ROOT = PROJECT_DIR.parent
HARDWARE_YAML = REPO_ROOT / "ros2_ws/src/robot_bringup/config/hardware.yaml"

# Firmware constants to extract and their Python types.
CONSTANTS: dict[str, type] = {
    "BAUD_RATE": int,
    "TRACK_WIDTH_M": float,
}


def extract_const(decls: dict, name: str, typ: type) -> int | float:
    """Extract the initializer value of a const variable from its token stream.

    Reads the tokens for the declaration (e.g. `const float TRACK_WIDTH_M = 0.15f ;`),
    finds `=`, and parses the following token. Strips C literal suffixes (f, u, l, etc.).
    Works for simple literal initializers; does not handle macro expressions.
    """
    var_decl = clang.cindex.CursorKind.VAR_DECL  # ty: ignore[unresolved-attribute]
    cursor = decls.get((name, var_decl))
    if cursor is None:
        raise SystemExit(f"Couldn't find const variable {name!r} in {CONFIG_HPP}")
    tokens = [t.spelling for t in cursor.get_tokens()]
    try:
        eq_idx = tokens.index("=")
    except ValueError:
        raise SystemExit(f"No initializer found for {name!r} — macro expression?")
    raw = tokens[eq_idx + 1].rstrip("fFuUlL")
    return typ(raw)


def _fmt(value: int | float) -> str:
    """Format a number cleanly — floats use 6 significant figures, dropping trailing zeros."""
    return str(value) if isinstance(value, int) else f"{value:.6g}"


def render(baud_rate: int, track_width: float) -> str:
    return f"""\
# AUTO-GENERATED — DO NOT EDIT BY HAND.
# baud_rate and track_width are extracted from config.hpp by generate_hardware_config.py.
# Run `just generate-config` to regenerate after changing config.hpp.
#
# emergency_stop_distance and scan_min_range are not in config.hpp.
# To change them, edit the render() function in generate_hardware_config.py and regenerate.
#
# serial_port is not set here — robot.launch.py detects it at launch time via
# detect_serial_ports(), since USB enumeration order isn't stable across replugs.

motor_control_bridge:
  ros__parameters:
    baud_rate: {_fmt(baud_rate)}  # mirrors BAUD_RATE in config.hpp
    track_width: {_fmt(track_width)}  # mirrors TRACK_WIDTH_M in config.hpp — TODO: calibrate

robot_controller:
  ros__parameters:
    emergency_stop_distance: 0.30  # meters

scan_filter:
  ros__parameters:
    scan_min_range: 0.27  # meters; lidar returns closer than this are the robot's own body
"""


def main():
    tu = parse_firmware()
    target = os.path.realpath(str(CONFIG_HPP))
    decls = collect_decls(tu, target)

    values = {name: extract_const(decls, name, typ) for name, typ in CONSTANTS.items()}
    for name, value in values.items():
        print(f"  {name} = {_fmt(value)}")

    output = render(
        baud_rate=int(values["BAUD_RATE"]), track_width=float(values["TRACK_WIDTH_M"])
    )
    HARDWARE_YAML.write_text(output)
    print(f"Wrote {HARDWARE_YAML}")


if __name__ == "__main__":
    main()
