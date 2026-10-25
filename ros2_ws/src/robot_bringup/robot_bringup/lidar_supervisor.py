"""Runs the lidar driver and restarts it on a freshly found port whenever it exits.

The driver quits when the lidar's USB connection drops, and the device comes back under
a new ttyUSB number, so a plain launch respawn would reopen a dead path.
"""

import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix

from robot_bringup.detect_ports import LIDAR_DRIVER, find_port

RESTART_DELAY_S = 2.0


def main():
    params_file = sys.argv[1]
    driver = (
        get_package_prefix("ldlidar_sl_ros2")
        + "/lib/ldlidar_sl_ros2/ldlidar_sl_ros2_node"
    )
    child = None
    stopping = False

    def stop(signum, _frame):
        nonlocal stopping
        stopping = True
        if child is not None:
            child.send_signal(signum)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    while not stopping:
        try:
            port = find_port(LIDAR_DRIVER)
        except RuntimeError as e:
            print(f"[lidar_supervisor] {e}; retrying", flush=True)
            time.sleep(RESTART_DELAY_S)
            continue

        print(f"[lidar_supervisor] starting driver on {port}", flush=True)
        child = subprocess.Popen(
            [
                driver,
                "--ros-args",
                "-r",
                "__node:=ldlidar_publisher_ld14",
                "--params-file",
                params_file,
                "-p",
                f"port_name:={port}",
            ]
        )
        code = child.wait()
        child = None
        if not stopping:
            print(f"[lidar_supervisor] driver exited ({code}); restarting", flush=True)
            time.sleep(RESTART_DELAY_S)


if __name__ == "__main__":
    main()
