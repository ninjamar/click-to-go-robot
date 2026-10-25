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

docker-shell:
    docker exec -it ros2 bash

# rebuild colcon packages inside the running container after source changes. Our Python
# packages are cleaned first: after the container is recreated, an incremental build can
# leave the image's stale install in place.
colcon-build:
    docker exec ros2 bash -c "cd /workspace/ros2_ws && rm -rf build/robot_bringup build/motor_control_bridge build/robot_controller install/robot_bringup install/motor_control_bridge install/robot_controller && source /opt/ros/jazzy/setup.bash && colcon build"

# launch the full navigation stack (lidar + SLAM + Nav2 + bridge + controller); refuses to
# start a second copy, which would fight over the serial ports and node names. Ctrl-C only
# stops the docker exec client, so the trap sends the interrupt into the container too.
launch:
    #!/usr/bin/env bash
    if docker exec ros2 pgrep -f 'robot_bringup robot.launch.py' >/dev/null; then
        echo "a launch is already running in the container; run 'just pkill' first" >&2
        exit 1
    fi
    trap "docker exec ros2 pkill -INT -f 'robot_bringup robot.launch.py'" INT TERM
    docker exec ros2 bash -c "source /workspace/ros2_ws/install/setup.bash && ros2 launch robot_bringup robot.launch.py"

# launch foxglove_bridge so Foxglove Studio can connect at ws://localhost:8765
foxglove-bridge:
    #!/usr/bin/env bash
    trap "docker exec ros2 pkill -INT -f 'foxglove_bridge_launch.xml'" INT TERM
    docker exec ros2 bash -c "source /workspace/ros2_ws/install/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml"

# stop everything ROS in the container: SIGINT to the launches (a clean shutdown),
# wait up to 5 s, then SIGKILL stragglers. The script comes in on stdin so that the
# patterns can't match this command's own command line.
pkill:
    #!/usr/bin/env bash
    docker exec -i ros2 bash -s <<'EOF'
    pkill -INT -f 'ros2 launch'
    for _ in $(seq 10); do
        pgrep -f 'ros2_ws|/opt/ros/jazzy' >/dev/null || exit 0
        sleep 0.5
    done
    pkill -9 -f 'ros2_ws|/opt/ros/jazzy'
    exit 0
    EOF

# attach every USB device listed by `orb usb list` that isn't already attached
usb-attach:
    #!/usr/bin/env zsh
    orb usb list | awk '$NF != "attached" {print $1}' | while read -r id; do
        orb usb attach "$id"
    done
    
# detach every USB device listed by `orb usb list` that isn't already attached
usb-detach:
    #!/usr/bin/env zsh
    orb usb list | awk '$NF == "attached" {print $1}' | while read -r id; do
        orb usb detach "$id"
    done

# USB serial devices the VM sees, with kernel driver (cp210x = lidar, ch341-uart = Nano); empty = none reached the VM
usb-devices:
    docker run --rm -v /sys:/sys:ro busybox sh -c 'for t in /sys/class/tty/ttyUSB*; do [ -e "$t" ] && echo "$(basename $t) $(cat $t/dev) $(basename $(readlink -f $t/device/driver))"; done; true'