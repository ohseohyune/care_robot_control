"""
neck_controller_node.py
=======================
Dynamixel XC330-T181-T position controller for neck yaw axis.

Hardware interface
------------------
Protocol  : Dynamixel Protocol 2.0 (TTL Half-Duplex)
Baud rate : 57600 (default; change to 1M for production)
ID        : 1 (configurable via parameter)
Mode      : Position Control Mode (joint mode, 0~360°)

Position conversion
-------------------
XC330 resolution: 4096 pulse/rev → 0.0879°/pulse = 0.001534 rad/pulse
Zero position    : 2048 (= 180° in absolute encoder, mapped to 0° in our frame)
Range            : ±60° = ±1.047 rad → [2048 - 682, 2048 + 682] = [1366, 2730]

This node subscribes to /neck_yaw_target [rad] and writes position to Dynamixel.
It also publishes /neck_yaw_state [rad] (current position feedback).

Dependencies
------------
pip install dynamixel-sdk

Simulation mode
---------------
If 'simulation_mode' parameter is true (default), skips hardware and just
echoes target as state — for testing without physical motor.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64


# Dynamixel XC330 control table addresses (Protocol 2.0)
ADDR_TORQUE_ENABLE      = 64
ADDR_OPERATING_MODE     = 11
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_PROFILE_VELOCITY   = 112   # [0.229 rpm/unit]
ADDR_PROFILE_ACCELERATION = 108

# XC330 constants
RESOLUTION         = 4096          # pulses per revolution
CENTER_PULSE       = 2048          # pulse value for 0 rad (center)
RAD_PER_PULSE      = 2 * math.pi / RESOLUTION   # ~0.001534 rad
OPERATING_MODE_POS = 3             # Position Control Mode

# Velocity profile: limit to 30 RPM for smooth motion
# Unit: 0.229 rpm → 30 rpm / 0.229 = ~131
PROFILE_VELOCITY   = 131
PROFILE_ACCEL      = 20            # arbitrary smooth ramp


def rad_to_pulse(rad: float) -> int:
    """Convert radians (±π) to XC330 absolute position pulse [0, 4095]."""
    pulse = CENTER_PULSE + int(round(rad / RAD_PER_PULSE))
    return max(0, min(4095, pulse))


def pulse_to_rad(pulse: int) -> float:
    """Convert XC330 absolute position pulse to radians."""
    return (pulse - CENTER_PULSE) * RAD_PER_PULSE


class NeckControllerNode(Node):
    def __init__(self):
        super().__init__("neck_controller_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("neck_limit", 1.047)   # [rad] ±60°
        self.declare_parameter("publish_hz", 50.0)

        self.sim_mode   = self.get_parameter("simulation_mode").value
        self.device     = self.get_parameter("device_name").value
        self.baud       = self.get_parameter("baud_rate").value
        self.dxl_id     = self.get_parameter("dxl_id").value
        self.neck_lim   = self.get_parameter("neck_limit").value
        pub_hz          = self.get_parameter("publish_hz").value

        # ── QoS ──────────────────────────────────────────────────────────
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers / Subscribers ──────────────────────────────────────
        self.sub_target = self.create_subscription(
            Float64, "/neck_yaw_target", self._target_cb, reliable_qos
        )
        self.pub_state = self.create_publisher(
            Float64, "/neck_yaw_state", reliable_qos
        )

        # ── Internal state ────────────────────────────────────────────────
        self.target_rad   = 0.0
        self.current_rad  = 0.0
        self.port_handler = None
        self.packet_handler = None

        # ── Hardware init ─────────────────────────────────────────────────
        if not self.sim_mode:
            self._init_dynamixel()
        else:
            self.get_logger().warn(
                "NeckControllerNode: SIMULATION MODE — no hardware communication"
            )

        # ── Timer ─────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / pub_hz, self._control_loop)

        self.get_logger().info(
            f"NeckControllerNode ready | ID={self.dxl_id} | "
            f"limit=±{math.degrees(self.neck_lim):.0f}° | sim={self.sim_mode}"
        )

    # ─────────────────────────────────────────────
    # Hardware init
    # ─────────────────────────────────────────────
    def _init_dynamixel(self):
        try:
            from dynamixel_sdk import (
                PortHandler, PacketHandler, COMM_SUCCESS
            )
        except ImportError:
            self.get_logger().error(
                "dynamixel_sdk not installed. Run: pip install dynamixel-sdk"
            )
            self.sim_mode = True
            return

        from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

        self.port_handler   = PortHandler(self.device)
        self.packet_handler = PacketHandler(2.0)   # Protocol 2.0

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port {self.device}")
            self.sim_mode = True
            return

        if not self.port_handler.setBaudRate(self.baud):
            self.get_logger().error("Failed to set baud rate")
            self.sim_mode = True
            return

        # Set operating mode to Position Control
        self._write1(ADDR_TORQUE_ENABLE, 0)                   # torque off first
        self._write1(ADDR_OPERATING_MODE, OPERATING_MODE_POS)
        self._write4(ADDR_PROFILE_VELOCITY, PROFILE_VELOCITY)
        self._write4(ADDR_PROFILE_ACCELERATION, PROFILE_ACCEL)
        self._write1(ADDR_TORQUE_ENABLE, 1)                   # torque on

        self.get_logger().info(
            f"Dynamixel XC330 (ID={self.dxl_id}) initialized on {self.device}"
        )

    def _write1(self, addr, value):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, addr, value
        )

    def _write4(self, addr, value):
        self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, addr, value
        )

    def _read4(self, addr):
        val, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, addr
        )
        return val

    # ─────────────────────────────────────────────
    # Callback
    # ─────────────────────────────────────────────
    def _target_cb(self, msg: Float64):
        rad = float(msg.data)
        self.target_rad = max(-self.neck_lim, min(self.neck_lim, rad))

    # ─────────────────────────────────────────────
    # Control loop
    # ─────────────────────────────────────────────
    def _control_loop(self):
        if self.sim_mode:
            # In simulation, treat target as current (instant response)
            self.current_rad = self.target_rad
        else:
            # Write goal position
            goal_pulse = rad_to_pulse(self.target_rad)
            self._write4(ADDR_GOAL_POSITION, goal_pulse)

            # Read present position
            present_pulse = self._read4(ADDR_PRESENT_POSITION)
            self.current_rad = pulse_to_rad(present_pulse)

        # Publish current state
        state_msg = Float64()
        state_msg.data = self.current_rad
        self.pub_state.publish(state_msg)

    # ─────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────
    def destroy_node(self):
        if self.port_handler is not None:
            # Disable torque before closing
            self._write1(ADDR_TORQUE_ENABLE, 0)
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NeckControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# ─────────────────────────────────────────────────────────────────────────────
# NOTES
# ─────────────────────────────────────────────────────────────────────────────
# XC330 default ID is 1. Use Dynamixel Wizard to verify / change ID.
#
# For Jazzy, the official dynamixel_hardware package is available:
#   https://github.com/ROBOTIS-GIT/dynamixel_hardware
# This node is a minimal standalone alternative — if you use ros2_control,
# replace this with the dynamixel_hardware plugin and a joint_trajectory_controller.
#
# Baud rate: default 57600. For 4 Mbps production use:
#   self.port_handler.setBaudRate(4000000)
#   and set EEPROM baud rate via Dynamixel Wizard.
# ─────────────────────────────────────────────────────────────────────────────