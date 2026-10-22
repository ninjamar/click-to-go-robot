set default-list := true

format:
    #!/usr/bin/env zsh

    # isort .
    ruff check --select I --fix .

    ruff format . 
    clang-format -i $(fd -e cpp -e hpp . motor_control_system) # fd respects gitignore; the dot is the pattern (find all)


compiledb:
    @test -f motor_control_system/compile_commands.json || (cd motor_control_system && pio run -t compiledb)

update: _clean compiledb

_clean:
    rm -f motor_control_system/compile_commands.json

lint: compiledb
    #!/usr/bin/env zsh

    ruff check .
    ty check .
    clang-tidy -p motor_control_system motor_control_system/src/main.cpp


generate-protocol: compiledb
    cd motor_control_system/py && python -m scripts.codegen.generate_protocol

generate-config: compiledb
    cd motor_control_system/py && python -m scripts.codegen.generate_protocol
    cd motor_control_system/py && python -m scripts.codegen.generate_hardware_config

generate: generate-protocol generate-config

[no-exit-message]
@mcs-send-command *args:
    cd motor_control_system/py && python -m scripts.send_command {{args}}

[no-exit-message]
@mcs-test *args:
    pytest motor_control_system/py/tests {{args}}

docker-up:
    docker compose -f ros2_ws/compose.yaml up -d

# rebuilds the image (use after changing Dockerfile or apt deps)
docker-rebuild:
    docker compose -f ros2_ws/compose.yaml up --build -d

# use when hardware is actually plugged in and attached via `orb usb attach`
docker-up-hardware:
    docker compose -f ros2_ws/compose.yaml -f ros2_ws/compose.hardware.yaml up -d

docker-shell:
    docker exec -it ros2 bash

# rebuild colcon packages inside the running container after source changes
colcon-build:
    docker exec ros2 bash -c "source /opt/ros/jazzy/setup.bash && cd /workspace/ros2_ws && colcon build"

# launch the full navigation stack (lidar + SLAM + Nav2 + bridge + controller)
launch:
    docker exec ros2 bash -c "source /workspace/ros2_ws/install/setup.bash && ros2 launch robot_bringup robot.launch.py"

# launch foxglove_bridge so Foxglove Studio can connect at ws://localhost:8765
foxglove-bridge:
    docker exec ros2 bash -c "source /workspace/ros2_ws/install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

pkill:
    docker exec ros2 bash -c "pkill -f ros"

# attach every USB device listed by `orb usb list` that isn't already attached
usb-attach:
    #!/usr/bin/env zsh
    orb usb list | awk '$NF != "attached" {print $1}' | while read -r id; do
        orb usb attach "$id"
    done

# lists the device paths Docker actually sees, for wiring up compose.hardware.yaml
usb-devices:
    docker run --rm -v /dev:/dev busybox sh -c 'ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null'