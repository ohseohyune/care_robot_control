"""
sim_launch.py
=============
Launch all 3 nodes for software-in-the-loop simulation.

Usage:
    ros2 launch care_robot_control sim_launch.py
    ros2 launch care_robot_control sim_launch.py person_mode:=circle
    ros2 launch care_robot_control sim_launch.py person_mode:=linear person_speed:=0.5

Then monitor:
    ros2 topic echo /robot_state
    ros2 topic echo /person_detection_raw
    ros2 topic hz /cmd_vel

Visualize with the matplotlib visualizer (separate script):
    python3 visualizer.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments (overridable from CLI) ───────────────────────────
    person_mode_arg  = DeclareLaunchArgument("person_mode",  default_value="circle")
    person_speed_arg = DeclareLaunchArgument("person_speed", default_value="0.3")
    person_radius_arg = DeclareLaunchArgument("person_radius", default_value="2.0")

    person_mode   = LaunchConfiguration("person_mode")
    person_speed  = LaunchConfiguration("person_speed")
    person_radius = LaunchConfiguration("person_radius")

    # ── Node: Robot + Vision Simulator ───────────────────────────────────
    simulator_node = Node(
        package="care_robot_control",
        executable="robot_simulator",
        name="robot_simulator_node",
        output="screen",
        parameters=[{
            "sim_dt":           0.005,         # 200 Hz integration
            "publish_dt":       0.0667,        # 15 Hz vision output
            "noise_dist_std":   0.03,
            "noise_angle_std":  0.02,
            "person_mode":      person_mode,
            "person_speed":     person_speed,
            "person_radius":    person_radius,
            "motor_tau":        0.08,
            "wheel_radius":     0.0325,
            "wheel_base":       0.2115,
            "neck_offset_x":    0.1244,
            "detection_range":  5.0,
        }],
    )

    # ── Node: Neck Controller (simulation mode) ───────────────────────────
    neck_node = Node(
        package="care_robot_control",
        executable="neck_controller",
        name="neck_controller_node",
        output="screen",
        parameters=[{
            "simulation_mode": True,    # no hardware
            "neck_limit":      1.047,   # ±60°
            "publish_hz":      50.0,
        }],
    )

    # ── Node: Person Follower Controller ─────────────────────────────────
    follower_node = Node(
        package="care_robot_control",
        executable="person_follower",
        name="person_follower_node",
        output="screen",
        parameters=[{
            "wheel_radius":     0.0325,
            "wheel_base":       0.2115,
            "target_distance":  1.0,
            "distance_tol":     0.10,
            "Kp_v":             0.6,
            "Kd_v":             0.15,
            "Kp_w":             1.2,
            "Kd_w":             0.05,
            "v_max":            0.4,
            "omega_max":        1.0,
            "neck_limit":       1.047,
            "neck_return_gain": 0.3,
            "camera_fov_h":     2.094,
            "search_speed":     0.4,
            "search_timeout":   5.0,
            "vision_timeout":   0.5,
            "control_hz":       50.0,
        }],
    )

    return LaunchDescription([
        person_mode_arg,
        person_speed_arg,
        person_radius_arg,
        simulator_node,
        neck_node,
        follower_node,
    ])
