"""Shared libclang utilities for parsing the PlatformIO firmware.

Requires compile_commands.json — run `pio run -t compiledb` (or `just compiledb`) first.
"""

import glob
import json
import os
import shlex
from pathlib import Path

import clang.cindex

PROJECT_DIR = Path(__file__).parents[3]  # motor_control_system/
COMPILE_COMMANDS = PROJECT_DIR / "compile_commands.json"
MAIN_CPP = "src/main.cpp"


def load_compile_args() -> tuple[str, list[str]]:
    """Return (directory, clang_args) derived from PlatformIO's compile_commands.json.

    Patches the raw AVR compiler invocation so libclang (which isn't avr-g++) can
    parse firmware headers: adds --target=avr and the AVR toolchain's builtin include
    dirs that avr-g++ would normally provide implicitly.
    """
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
    # Derive the toolchain root from the compiler path so this works regardless
    # of PLATFORMIO_CORE_DIR.
    toolchain = Path(tokens[0]).parent.parent
    if not toolchain.exists():
        raise SystemExit(f"AVR toolchain not found at {toolchain}")

    args = []
    i = 1  # skip compiler executable
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

    gcc_builtin_include = glob.glob(str(toolchain / "lib/gcc/avr/*/include"))
    if not gcc_builtin_include:
        raise SystemExit(f"No gcc builtin include dir found under {toolchain}")

    extra_includes = gcc_builtin_include + [str(toolchain / "avr/include")]
    args = ["--target=avr"] + [f"-I{d}" for d in extra_includes] + args
    return entry["directory"], args


def parse_firmware() -> clang.cindex.TranslationUnit:
    """Parse main.cpp with AVR-aware flags; raise SystemExit on any errors."""
    directory, args = load_compile_args()
    os.chdir(directory)
    index = clang.cindex.Index.create()
    tu = index.parse(MAIN_CPP, args=args)
    errors = [d for d in tu.diagnostics if d.severity >= clang.cindex.Diagnostic.Error]
    if errors:
        raise SystemExit("\n".join(str(d) for d in errors))
    return tu


def collect_decls(tu: clang.cindex.TranslationUnit, target_file: str) -> dict:
    """Walk the AST once and return declarations from target_file keyed by (name, kind)."""
    decls = {}
    for cursor in tu.cursor.walk_preorder():
        if (
            cursor.location.file
            and os.path.realpath(str(cursor.location.file)) == target_file
        ):
            decls[(cursor.spelling, cursor.kind)] = cursor
    return decls


def find_decl(decls: dict, name: str, kind) -> clang.cindex.Cursor:
    """Look up a declaration by name and kind; raise SystemExit if not found."""
    cursor = decls.get((name, kind))
    if cursor is None:
        raise SystemExit(f"Couldn't find {kind} {name!r}")
    return cursor
