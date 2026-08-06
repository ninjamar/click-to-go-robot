"""
Generate protocol.py from serial_protocol.hpp via a real libclang parse.

Usage: just generate-protocol (requires compile_commands.json - run `pio run -t compiledb` first)
"""

import os
from pathlib import Path

import clang.cindex

from .firmware_parse import collect_decls, find_decl, parse_firmware

PROJECT_DIR = Path(__file__).parents[3]  # motor_control_system/
SERIAL_PROTOCOL_HPP = PROJECT_DIR / "lib/serial_protocol/serial_protocol.hpp"
OUTPUT_FILE = (
    Path(__file__).parents[2] / "protocol" / "protocol.py"
)  # py/protocol/protocol.py

TYPE_MAP = {
    "uint8_t": "B",
    "int8_t": "b",
    "uint16_t": "H",
    "float": "f",
}


def is_queued(op_name):
    """Mirrors the enum's own naming convention instead of hand-maintaining a second list."""
    return "QUEUE" in op_name


def cli_name(op_name):
    """CLI-friendly spelling for an op: drop the QUEUE_ prefix, lowercase
    the rest (OP_QUEUE_MOVE_DIST -> move_dist, OP_DEBUG_SET_PWM -> debug_set_pwm)."""
    return op_name.replace("QUEUE_", "", 1).lower()  # todo: make lower case keep queue


def extract_enum(cursor, prefix):
    members = []
    for child in cursor.get_children():
        if child.kind == clang.cindex.CursorKind.ENUM_CONSTANT_DECL:  # ty: ignore[unresolved-attribute]
            if not child.spelling.startswith(prefix):
                raise SystemExit(f"{child.spelling!r} doesn't start with {prefix!r}")
            members.append((child.spelling[len(prefix) :], child.enum_value))
    return members


def extract_struct_fields(cursor):
    attrs = [c.kind for c in cursor.get_children()]
    if clang.cindex.CursorKind.PACKED_ATTR not in attrs:  # ty: ignore[unresolved-attribute]
        raise SystemExit(f"struct {cursor.spelling} isn't __attribute__((packed))")

    fields = []
    for child in cursor.get_children():
        if child.kind != clang.cindex.CursorKind.FIELD_DECL:  # ty: ignore[unresolved-attribute]
            continue
        c_type = child.type.spelling
        if c_type not in TYPE_MAP:
            raise SystemExit(
                f"{cursor.spelling}.{child.spelling}: unsupported type {c_type!r} "
                f"(add it to TYPE_MAP in {Path(__file__).name})"
            )
        fields.append((child.spelling, TYPE_MAP[c_type]))
    return fields


def split_off_checksum(fields, struct_name):
    """Both packets end with a CRC-16/MODBUS checksum field, computed/verified
    rather than supplied by callers - split it off from the payload fields."""
    if fields[-1] != ("checksum", "H"):
        raise SystemExit(f"{struct_name} must end with a uint16_t checksum field")
    return fields[:-1]


def render(op_members, status_members, command_fields, response_fields):
    op_lines = "\n".join(f"    {name} = {value}" for name, value in op_members)
    status_lines = "\n".join(f"    {name} = {value}" for name, value in status_members)

    queued_ops = ", ".join(f"OP.{name}" for name, _ in op_members if is_queued(name))
    cli_names = "\n".join(
        f'    "{cli_name(name)}": OP.{name},' for name, _ in op_members
    )

    command_payload_fields = split_off_checksum(command_fields, "CommandPacket")
    command_payload_format = "<" + "".join(fmt for _, fmt in command_payload_fields)
    command_params = ", ".join(name for name, _ in command_payload_fields)

    response_payload_fields = split_off_checksum(response_fields, "ResponsePacket")
    response_payload_format = "<" + "".join(fmt for _, fmt in response_payload_fields)
    response_format = response_payload_format + "H"
    response_names = [name for name, _ in response_payload_fields]
    response_names_literal = ", ".join(f'"{name}"' for name in response_names)
    return_values = ", ".join(
        f"Status({name})" if name == "status" else name for name in response_names
    )

    return f'''# AUTO-GENERATED FILE - DO NOT EDIT BY HAND.
# Run `just generate-protocol` to regenerate from serial_protocol.hpp.

# ruff: noqa
# isort: skip_file
# fmt: off

import struct
from collections import namedtuple
from enum import IntEnum

from fastcrc import crc16

class OP(IntEnum):
{op_lines}


class Status(IntEnum):
{status_lines}


QUEUED_OPS = frozenset({{{queued_ops}}})

CLI_NAMES = {{
{cli_names}
}}


def build_command_packet({command_params}):
    payload = struct.pack("{command_payload_format}", {command_params})
    checksum = crc16.modbus(payload)
    return payload + struct.pack("<H", checksum)


RESPONSE_PAYLOAD_SIZE = struct.calcsize("{response_payload_format}")
RESPONSE_FORMAT = "{response_format}"
RESPONSE_SIZE = struct.calcsize(RESPONSE_FORMAT)

ResponsePacket = namedtuple("ResponsePacket", [{response_names_literal}])


def parse_response_packet(data):
    data = data[:RESPONSE_SIZE]
    {", ".join(response_names)}, checksum = struct.unpack(
        RESPONSE_FORMAT, data
    )
    if crc16.modbus(data[:RESPONSE_PAYLOAD_SIZE]) != checksum:
        raise ValueError(f"checksum mismatch in response {{data!r}}")
    return ResponsePacket({return_values})

# fmt: on
'''


def main():
    tu = parse_firmware()

    target_file = os.path.realpath(str(SERIAL_PROTOCOL_HPP))
    enum_decl = clang.cindex.CursorKind.ENUM_DECL  # ty: ignore[unresolved-attribute]
    struct_decl = clang.cindex.CursorKind.STRUCT_DECL  # ty: ignore[unresolved-attribute]

    decls = collect_decls(tu, target_file)
    op_cursor = find_decl(decls, "CommandOperation", enum_decl)
    status_cursor = find_decl(decls, "ResponseStatus", enum_decl)
    command_cursor = find_decl(decls, "CommandPacket", struct_decl)
    response_cursor = find_decl(decls, "ResponsePacket", struct_decl)

    op_members = extract_enum(op_cursor, "OP_")
    status_members = extract_enum(status_cursor, "STATUS_")
    command_fields = extract_struct_fields(command_cursor)
    response_fields = extract_struct_fields(response_cursor)

    output = render(op_members, status_members, command_fields, response_fields)
    OUTPUT_FILE.write_text(output)
    print(f"Wrote {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
