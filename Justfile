set default-list := true

format:
    #!/usr/bin/env zsh

    ruff format . 
    isort .
    clang-format -i $(fd -e cpp -e hpp . motor_control_system) # fd respects gitignore; the dot is the pattern (find all)


compiledb:
    test -f motor_control_system/compile_commands.json || (cd motor_control_system && pio run -t compiledb)

update: _clean compiledb

_clean:
    rm -f motor_control_system/compile_commands.json

lint: compiledb
    #!/usr/bin/env zsh

    ruff check .
    ty check .
    clang-tidy -p motor_control_system motor_control_system/src/main.cpp

generate-protocol: compiledb
    python motor_control_system/py/scripts/generate_protocol.py