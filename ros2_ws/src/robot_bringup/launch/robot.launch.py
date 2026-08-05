import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from robot_bringup.detect_ports import detect_serial_ports


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory("robot_bringup")

    # hardware.yaml: physical constants (track width, robot radius, speeds).
    # tuning.yaml: algorithm params (SLAM settings, Nav2 critic weights, motion slices).
    # Our own nodes receive both; hardware values override tuning values on any overlap.
    # SLAM and Nav2 (via IncludeLaunchDescription) receive only tuning_params — their
    # launch files accept a single params file. Values marked "mirrors hardware.yaml"
    # in tuning.yaml must be kept in sync manually.
    hardware_params = os.path.join(bringup_dir, "config", "hardware.yaml")
    tuning_params = os.path.join(bringup_dir, "config", "tuning.yaml")

    # USB enumeration order isn't guaranteed across replugs, so identify the lidar
    # and the Nano by behavior rather than trusting a fixed /dev path.
    nano_port, lidar_port = detect_serial_ports()

    # Lidar driver: publishes /scan (frame: base_laser) and a static TF
    # base_link→base_laser at z=0.18 m. Defined directly (rather than including
    # ldlidar_sl_ros2's ld14p.launch.py) so the detected port can be injected —
    # its launch file hardcodes port_name with no way to override it.
    lidar_node = Node(
        package="ldlidar_sl_ros2",
        executable="ldlidar_sl_ros2_node",
        name="ldlidar_publisher_ld14",
        output="screen",
        parameters=[
            {"product_name": "LDLiDAR_LD14P"},
            {"laser_scan_topic_name": "scan"},
            {"point_cloud_2d_topic_name": "pointcloud2d"},
            {"frame_id": "base_laser"},
            {"port_name": lidar_port},
            {"serial_baudrate": 230400},
            # Lidar is mounted upside-down (180° roll), which reverses the physical
            # CW/CCW handedness of the raw scan relative to the sensor's own frame.
            {"laser_scan_dir": True},
            {"enable_angle_crop_func": False},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
        ],
    )
    # Lidar is physically flipped 180° about the robot's forward axis (roll),
    # so base_laser's axes no longer align with base_link's — roll=pi corrects it.
    lidar_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_ld14p",
        arguments=["0", "0", "0.18", "0", "0", "3.14159265", "base_link", "base_laser"],
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
            "slam_params_file": tuning_params,
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
            "params_file": tuning_params,
            "use_sim_time": "false",
        }.items(),
    )

    # Bridge: translates /cmd_vel into serial motor commands and publishes /odom +
    # TF odom→base_link from wheel encoder data. serial_port overrides whatever
    # hardware.yaml has on file with the port detected for this boot.
    bridge = Node(
        package="motor_control_bridge",
        executable="bridge_node",
        parameters=[tuning_params, hardware_params, {"serial_port": nano_port}],
        output="screen",
    )

    # Controller: accepts /goal_pose from RViz2 and drives Nav2's NavigateToPose action.
    controller = Node(
        package="robot_controller",
        executable="controller_node",
        parameters=[tuning_params, hardware_params],
        output="screen",
    )

    return [lidar_node, lidar_tf, slam, nav2, bridge, controller]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
