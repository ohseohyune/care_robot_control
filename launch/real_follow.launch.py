"""
real_follow.launch.py
=====================
목 모터 컨트롤러 + ebimu IMU 노드 런치 파일.
ebimu_node가 serial에서 읽어 senior_msg/ImuMsg를 Imu 토픽에 퍼블리시.
사람 추종 알고리즘은 seniorcare_robot/master_node에서 처리.

Example
-------
ros2 launch care_robot_control real_follow.launch.py \
    device_name:=/dev/ttyUSB0 baud_rate:=57600 dxl_id:=1 center_tick:=2048
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("ebimu_port",               default_value="/dev/ttyUSB-EBIMU"),
        DeclareLaunchArgument("ebimu_baudrate",           default_value="115200"),
        DeclareLaunchArgument("simulation_mode",          default_value="false"),
        DeclareLaunchArgument("device_name",              default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baud_rate",                default_value="57600"),
        DeclareLaunchArgument("dxl_id",                   default_value="1"),
        DeclareLaunchArgument("center_tick",              default_value="2048"),
        DeclareLaunchArgument("reverse_direction",        default_value="false"),
        DeclareLaunchArgument("neck_limit",               default_value="1.047"),
        DeclareLaunchArgument("publish_hz",               default_value="50.0"),
        DeclareLaunchArgument("profile_velocity_rpm",     default_value="25.0"),
        DeclareLaunchArgument("profile_acceleration_raw", default_value="20"),
        DeclareLaunchArgument("command_timeout",          default_value="0.5"),
        DeclareLaunchArgument("max_command_step",         default_value="0.08"),
    ]

    ebimu_node = Node(
        package="care_robot_control",
        executable="ebimu",
        name="ebimu_node",
        output="screen",
        parameters=[{
            "port_name": LaunchConfiguration("ebimu_port"),
            "baudrate":  LaunchConfiguration("ebimu_baudrate"),
        }],
    )

    neck_node = Node(
        package="care_robot_control",
        executable="neck_controller",
        name="neck_controller_node",
        output="screen",
        parameters=[{
            "simulation_mode":          LaunchConfiguration("simulation_mode"),
            "device_name":              LaunchConfiguration("device_name"),
            "baud_rate":                LaunchConfiguration("baud_rate"),
            "dxl_id":                   LaunchConfiguration("dxl_id"),
            "center_tick":              LaunchConfiguration("center_tick"),
            "reverse_direction":        LaunchConfiguration("reverse_direction"),
            "neck_limit":               LaunchConfiguration("neck_limit"),
            "publish_hz":               LaunchConfiguration("publish_hz"),
            "profile_velocity_rpm":     LaunchConfiguration("profile_velocity_rpm"),
            "profile_acceleration_raw": LaunchConfiguration("profile_acceleration_raw"),
            "command_timeout":          LaunchConfiguration("command_timeout"),
            "max_command_step":         LaunchConfiguration("max_command_step"),
            "park_on_shutdown": True,
            "shutdown_position": 0.0,
        }],
    )

    base_driver_node = Node(
        package="capstone_base_driver",
        executable="base_driver_node",
        name="capstone_base_driver",
        output="screen",
        parameters=[{
            "port":             "/dev/ttyACM0",
            "baudrate":         115200,
            "status_period_ms": 100,
        }],
    )

    return LaunchDescription(args + [ebimu_node, neck_node, base_driver_node])
