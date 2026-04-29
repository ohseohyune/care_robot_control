"""
real_follow.launch.py
=====================
Launch person follower + real Dynamixel neck controller for hardware tests.

Expected external nodes
-----------------------
- Vision node publishing /person_detection (or /person_detection_raw fallback)
- Base driver consuming /cmd_vel

Example
-------
ros2 launch care_robot_control real_follow.launch.py \
    device_name:=/dev/ttyUSB0 baud_rate:=57600 dxl_id:=1 center_tick:=2048
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("device_name", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate", default_value="57600"),
        DeclareLaunchArgument("dxl_id", default_value="1"),
        DeclareLaunchArgument("center_tick", default_value="2048"),
        DeclareLaunchArgument("reverse_direction", default_value="false"),
        DeclareLaunchArgument("neck_limit", default_value="1.047"),
        DeclareLaunchArgument("publish_hz", default_value="50.0"),
        DeclareLaunchArgument("profile_velocity_rpm", default_value="25.0"),
        DeclareLaunchArgument("profile_acceleration_raw", default_value="20"),
        DeclareLaunchArgument("command_timeout", default_value="0.5"),
        DeclareLaunchArgument("max_command_step", default_value="0.08"),
        DeclareLaunchArgument("target_distance", default_value="1.0"),
        DeclareLaunchArgument("distance_tol", default_value="0.05"),
        DeclareLaunchArgument("control_hz", default_value="50.0"),
        DeclareLaunchArgument("v_max", default_value="0.6"),
        DeclareLaunchArgument("omega_max", default_value="1.0"),
    ]

    device_name = LaunchConfiguration("device_name")
    baud_rate = LaunchConfiguration("baud_rate")
    dxl_id = LaunchConfiguration("dxl_id")
    center_tick = LaunchConfiguration("center_tick")
    reverse_direction = LaunchConfiguration("reverse_direction")
    neck_limit = LaunchConfiguration("neck_limit")
    publish_hz = LaunchConfiguration("publish_hz")
    profile_velocity_rpm = LaunchConfiguration("profile_velocity_rpm")
    profile_acceleration_raw = LaunchConfiguration("profile_acceleration_raw")
    command_timeout = LaunchConfiguration("command_timeout")
    max_command_step = LaunchConfiguration("max_command_step")
    target_distance = LaunchConfiguration("target_distance")
    distance_tol = LaunchConfiguration("distance_tol")
    control_hz = LaunchConfiguration("control_hz")
    v_max = LaunchConfiguration("v_max")
    omega_max = LaunchConfiguration("omega_max")

    neck_node = Node(
        package="care_robot_control",
        executable="neck_controller",
        name="neck_controller_node",
        output="screen",
        parameters=[{
            "simulation_mode": False,
            "device_name": device_name,
            "baud_rate": baud_rate,
            "dxl_id": dxl_id,
            "center_tick": center_tick,
            "reverse_direction": reverse_direction,
            "neck_limit": neck_limit,
            "publish_hz": publish_hz,
            "profile_velocity_rpm": profile_velocity_rpm,
            "profile_acceleration_raw": profile_acceleration_raw,
            "command_timeout": command_timeout,
            "max_command_step": max_command_step,
            "park_on_shutdown": True,
            "shutdown_position": 0.0,
        }],
    )

    follower_node = Node(
        package="care_robot_control",
        executable="person_follower",
        name="person_follower_node",
        output="screen",
        parameters=[{
            "wheel_radius": 0.0325,
            "wheel_base": 0.2115,
            "target_distance": target_distance,
            "distance_tol": distance_tol,
            "Kp_v": 1.2,
            "Ki_v": 0.4,
            "Kd_v": 0.15,
            "i_max_v": 0.5,
            "distance_filter_alpha": 0.35,
            "deadband_integral_decay": 0.92,
            "Kp_w": 1.2,
            "Kd_w": 0.05,
            "heading_slowdown_angle": 0.70,
            "v_max": v_max,
            "omega_max": omega_max,
            "neck_limit": neck_limit,
            "neck_return_gain": 0.3,
            "camera_fov_h": 2.094,
            "search_speed": 0.8,
            "search_omega": 0.6,
            "vision_timeout": 0.3,
            "control_hz": control_hz,
        }],
    )

    return LaunchDescription(
        args + [
            LogInfo(
                msg=(
                    "real_follow.launch.py expects an external vision node on "
                    "/person_detection (or /person_detection_raw) and a base driver on /cmd_vel."
                )
            ),
            neck_node,
            follower_node,
        ]
    )
