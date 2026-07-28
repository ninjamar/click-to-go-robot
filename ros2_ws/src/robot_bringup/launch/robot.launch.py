import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("robot_bringup")

    slam_params = os.path.join(bringup_dir, "config", "slam_params.yaml")
    nav2_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")
    bridge_params = os.path.join(
        get_package_share_directory("motor_control_bridge"),
        "config",
        "bridge_params.yaml",
    )
    controller_params = os.path.join(
        get_package_share_directory("robot_controller"),
        "config",
        "controller_params.yaml",
    )

    # Lidar driver: opens /dev/ttyUSB0, publishes /scan (frame: base_laser) and
    # a static TF base_link→base_laser at z=0.18 m.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ldlidar_sl_ros2"),
                "launch",
                "ld14p.launch.py",
            )
        )
    )

    # SLAM Toolbox: matches /scan against the growing map to build /map and publish
    # TF map→odom (the correction that prevents encoder drift from compounding).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "params_file": slam_params,
            "use_sim_time": "false",
        }.items(),
    )

    # Nav2: reads /map, /scan, and TF to plan paths and publish /cmd_vel.
    # navigation_launch.py starts all Nav2 lifecycle nodes and their lifecycle manager.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "params_file": nav2_params,
            "use_sim_time": "false",
        }.items(),
    )

    # Bridge: translates /cmd_vel into serial motor commands and publishes /odom +
    # TF odom→base_link from wheel encoder data. Opens /dev/ttyUSB1 (Nano).
    bridge = Node(
        package="motor_control_bridge",
        executable="bridge_node",
        parameters=[bridge_params],
        output="screen",
    )

    # Controller: accepts /goal_pose from RViz2 and drives Nav2's NavigateToPose action.
    controller = Node(
        package="robot_controller",
        executable="controller_node",
        parameters=[controller_params],
        output="screen",
    )

    return LaunchDescription([lidar, slam, nav2, bridge, controller])
