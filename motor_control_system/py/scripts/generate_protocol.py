"""
Generate protocol.py from serial_protocol.hpp via a real libclang parse.

Usage: just generate-protocol (requires compile_commands.json - run `pio run -t compiledb` first)
"""

import glob
import json
import os
import shlex
from pathlib import Path

import clang.cindex

SCRIPTS_DIR = Path(__file__).parent
PY_DIR = SCRIPTS_DIR.parent
PROJECT_DIR = PY_DIR.parent
COMPILE_COMMANDS = PROJECT_DIR / "compile_commands.json"
MAIN_CPP = "src/main.cpp"
SERIAL_PROTOCOL_HPP = "lib/serial_protocol/serial_protocol.hpp"
OUTPUT_FILE = PY_DIR / "protocol.py"

TYPE_MAP = {
    "uint8_t": "B",
    "int8_t": "b",
    "float": "f",
}


def load_compile_args():
    if not COMPILE_COMMANDS.exists():
        raise SystemExit(
            f"{COMPILE_COMMANDS} not found. Run `pio run -t compiledb` in "
            f"{PROJECT_DIR} first."
        )

    entries = json.loads(COMPILE_COMMANDS.read_text())
    entry = next((e for e in entries if e["file"] == MAIN_CPP), None)
    if entry is None:
        raise SystemExit(f"No compile_commands.json entry for {MAIN_CPP}")

    tokens = shlex.split(entry["command"])
    # Derive the toolchain root from tokens[0] (the compiler path) instead of
    # hardcoding it, so this works regardless of PLATFORMIO_CORE_DIR.
    toolchain = Path(tokens[0]).parent.parent
    if not toolchain.exists():
        raise SystemExit(f"AVR toolchain not found at {toolchain}")

    args = []
    i = 1  # skip the compiler executable itself
    while i < len(tokens):
        tok = tokens[i]
        if tok in ("-o", "-c"):
            i += 2 if tok == "-o" else 1
            continue
        if tok == tokens[-1]:  # trailing input file
            i += 1
            continue
        args.append(tok)
        i += 1

    # libclang isn't avr-g++, so it doesn't know avr-g++'s own builtin include dirs - add them explicitly.
    gcc_builtin_include = glob.glob(str(toolchain / "lib/gcc/avr/*/include"))
    if not gcc_builtin_include:
        raise SystemExit(f"No gcc builtin include dir found under {toolchain}")

    extra_includes = gcc_builtin_include + [str(toolchain / "avr/include")]
    args = ["--target=avr"] + [f"-I{d}" for d in extra_includes] + args
    return entry["directory"], args


def collect_decls(tu, target_file):
    """One pass over the whole AST, keyed by (name, kind), instead of a
    fresh walk_preorder() per lookup."""
    decls = {}
    for cursor in tu.cursor.walk_preorder():
        if (
            cursor.location.file
            and os.path.realpath(str(cursor.location.file)) == target_file
        ):
            decls[(cursor.spelling, cursor.kind)] = cursor
    return decls


def find_decl(decls, name, kind):
    cursor = decls.get((name, kind))
    if cursor is None:
        raise SystemExit(f"Couldn't find {kind} {name!r} in {SERIAL_PROTOCOL_HPP}")
    return cursor


def is_queued(op_name):
    """Mirrors the enum's own naming convention instead of hand-maintaining a second list."""
    return "QUEUE" in op_name


def cli_name(op_name):
    """CLI-friendly spelling for an op: drop the QUEUE_ prefix, lowercase
    the rest (OP_QUEUE_MOVE_DIST -> move_dist, OP_DEBUG_SET_PWM -> debug_set_pwm)."""
    return op_name.replace("QUEUE_", "", 1).lower()


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


def render(op_members, status_members, command_fields, response_fields):
    op_lines = "\n".join(f"    {name} = {value}" for name, value in op_members)
    status_lines = "\n".join(f"    {name} = {value}" for name, value in status_members)

    queued_ops = ", ".join(f"OP.{name}" for name, _ in op_members if is_queued(name))
    cli_names = "\n".join(
        f'    "{cli_name(name)}": OP.{name},' for name, _ in op_members
    )

    command_format = "<" + "".join(fmt for _, fmt in command_fields)
    command_params = ", ".join(name for name, _ in command_fields)

    response_format = "<" + "".join(fmt for _, fmt in response_fields)
    response_names = [name for name, _ in response_fields]
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


class OP(IntEnum):
{op_lines}


class Status(IntEnum):
{status_lines}


QUEUED_OPS = frozenset({{{queued_ops}}})

CLI_NAMES = {{
{cli_names}
}}


def build_command_packet({command_params}):
    return struct.pack("{command_format}", {command_params})


RESPONSE_FORMAT = "{response_format}"
RESPONSE_SIZE = struct.calcsize(RESPONSE_FORMAT)

ResponsePacket = namedtuple("ResponsePacket", [{response_names_literal}])


def parse_response_packet(data):
    {", ".join(response_names)} = struct.unpack(
        RESPONSE_FORMAT, data[:RESPONSE_SIZE]
    )
    return ResponsePacket({return_values})

# fmt: on
'''


def main():
    directory, args = load_compile_args()
    os.chdir(directory)

    index = clang.cindex.Index.create()
    tu = index.parse(MAIN_CPP, args=args)

    errors = [d for d in tu.diagnostics if d.severity >= clang.cindex.Diagnostic.Error]
    if errors:
        raise SystemExit("\n".join(str(d) for d in errors))

    target_file = os.path.realpath(SERIAL_PROTOCOL_HPP)
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
