"""
neck_controller_node.py
=======================
Hardware-facing neck yaw controller for Dynamixel XC330-T181-T.

ROS interface
-------------
Subscribes:
  /neck_yaw_target   std_msgs/Float64   desired neck yaw [rad]

Publishes:
  /neck_yaw_state    std_msgs/Float64   measured neck yaw [rad]

Features
--------
- Supports simulation passthrough mode for software-only testing
- Uses Dynamixel Protocol 2.0 over U2D2
- Converts ROS radians <-> Dynamixel position ticks
- Applies configurable direction, center tick, and soft joint limits
- Reads back present position at a fixed rate
- Optionally ramps commands to avoid abrupt neck motion
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_DISABLE = 0
TORQUE_ENABLE = 1
OPERATING_MODE_POSITION = 3

XC330_RESOLUTION = 4096
DEFAULT_CENTER_TICK = 2048
DEFAULT_PROTOCOL_VERSION = 2.0
DEFAULT_PROFILE_ACCEL_RAW = 20


class NeckControllerNode(Node):
    def __init__(self):
        super().__init__("neck_controller_node")

        self._declare_parameters()

        self.simulation_mode = self.get_parameter("simulation_mode").value
        self.device_name = self.get_parameter("device_name").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.protocol_version = float(self.get_parameter("protocol_version").value)
        self.dxl_id = int(self.get_parameter("dxl_id").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.neck_limit = float(self.get_parameter("neck_limit").value)
        self.center_tick = int(self.get_parameter("center_tick").value)
        self.reverse_direction = bool(self.get_parameter("reverse_direction").value)
        self.profile_velocity_rpm = float(
            self.get_parameter("profile_velocity_rpm").value
        )
        self.profile_acceleration_raw = int(
            self.get_parameter("profile_acceleration_raw").value
        )
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_command_step = float(self.get_parameter("max_command_step").value)
        self.park_on_shutdown = bool(self.get_parameter("park_on_shutdown").value)
        self.shutdown_position = float(
            self.get_parameter("shutdown_position").value
        )
        self.startup_read_retries = int(
            self.get_parameter("startup_read_retries").value
        )

        self.rad_per_tick = (2.0 * math.pi) / XC330_RESOLUTION

        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub_target = self.create_subscription(
            Float64, "/neck_yaw_target", self._target_cb, reliable_qos
        )
        self.pub_state = self.create_publisher(Float64, "/neck_yaw_state", reliable_qos)

        self.target_rad = 0.0
        self.commanded_rad = 0.0
        self.current_rad = 0.0
        self.last_target_time = None
        self.last_feedback_tick: Optional[int] = None

        self.port_handler = None
        self.packet_handler = None
        self.comm_success = None
        self.hw_ready = False
        self.hw_error_reported = False

        if self.simulation_mode:
            self.get_logger().warn(
                "Neck controller running in simulation_mode=True; hardware IO skipped."
            )
        else:
            self._init_dynamixel()

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self._control_loop)

        self.get_logger().info(
            "NeckControllerNode ready | "
            f"id={self.dxl_id} port={self.device_name} baud={self.baud_rate} "
            f"sim={self.simulation_mode} limit=+-{math.degrees(self.neck_limit):.1f}deg"
        )

    def _declare_parameters(self):
        self.declare_parameter("simulation_mode", True)
        self.declare_parameter("device_name", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)
        self.declare_parameter("protocol_version", DEFAULT_PROTOCOL_VERSION)
        self.declare_parameter("dxl_id", 1)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("neck_limit", 1.047)
        self.declare_parameter("center_tick", DEFAULT_CENTER_TICK)
        self.declare_parameter("reverse_direction", False)
        self.declare_parameter("profile_velocity_rpm", 30.0)
        self.declare_parameter("profile_acceleration_raw", DEFAULT_PROFILE_ACCEL_RAW)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("max_command_step", 0.08)
        self.declare_parameter("park_on_shutdown", True)
        self.declare_parameter("shutdown_position", 0.0)
        self.declare_parameter("startup_read_retries", 5)

    def _target_cb(self, msg: Float64):
        self.target_rad = self._clamp_rad(float(msg.data))
        self.last_target_time = self.get_clock().now()

    def _control_loop(self):
        if self.simulation_mode:
            self.commanded_rad = self._slew_toward(self.commanded_rad, self.target_rad)
            self.current_rad = self.commanded_rad
            self._publish_state()
            return

        if not self.hw_ready:
            if not self.hw_error_reported:
                self.get_logger().error(
                    "Dynamixel hardware is not ready. Check launch logs for the first "
                    "port/baud/ID error and verify the motor power is on."
                )
                self.hw_error_reported = True
            return

        desired_rad = self._get_live_target()
        self.commanded_rad = self._slew_toward(self.commanded_rad, desired_rad)

        goal_tick = self._rad_to_tick(self.commanded_rad)
        if self._write4(ADDR_GOAL_POSITION, goal_tick):
            self.last_feedback_tick = goal_tick

        present_tick = self._read_present_position()
        if present_tick is not None:
            self.current_rad = self._tick_to_rad(present_tick)

        self._publish_state()

    def _get_live_target(self) -> float:
        if self.last_target_time is None or self.command_timeout <= 0.0:
            return self.target_rad

        age = (self.get_clock().now() - self.last_target_time).nanoseconds * 1e-9
        if age <= self.command_timeout:
            return self.target_rad

        return 0.0

    def _slew_toward(self, current: float, target: float) -> float:
        max_step = max(self.max_command_step, 0.0)
        if max_step <= 0.0:
            return target
        delta = max(-max_step, min(max_step, target - current))
        return self._clamp_rad(current + delta)

    def _clamp_rad(self, rad: float) -> float:
        return max(-self.neck_limit, min(self.neck_limit, rad))

    def _rad_to_tick(self, rad: float) -> int:
        rad = self._clamp_rad(rad)
        sign = -1 if self.reverse_direction else 1
        tick = self.center_tick + int(round(sign * rad / self.rad_per_tick))
        return max(0, min(XC330_RESOLUTION - 1, tick))

    def _tick_to_rad(self, tick: int) -> float:
        sign = -1 if self.reverse_direction else 1
        rad = sign * (tick - self.center_tick) * self.rad_per_tick
        return self._clamp_rad(rad)

    def _publish_state(self):
        msg = Float64()
        msg.data = self.current_rad
        self.pub_state.publish(msg)

    def _init_dynamixel(self):
        try:
            from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
        except ImportError:
            self.get_logger().error(
                "Missing dynamixel_sdk. Install it with `pip install dynamixel-sdk`."
            )
            return

        self.comm_success = COMM_SUCCESS
        self.port_handler = PortHandler(self.device_name)
        self.packet_handler = PacketHandler(self.protocol_version)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open Dynamixel port {self.device_name}")
            return

        if not self.port_handler.setBaudRate(self.baud_rate):
            self.get_logger().error(
                f"Failed to set baud rate {self.baud_rate} on {self.device_name}"
            )
            self.port_handler.closePort()
            self.port_handler = None
            return

        if not self._write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE):
            return
        if not self._write1(ADDR_OPERATING_MODE, OPERATING_MODE_POSITION):
            return

        velocity_raw = max(1, int(round(self.profile_velocity_rpm / 0.229)))
        self._write4(ADDR_PROFILE_ACCELERATION, self.profile_acceleration_raw)
        self._write4(ADDR_PROFILE_VELOCITY, velocity_raw)

        if not self._write1(ADDR_TORQUE_ENABLE, TORQUE_ENABLE):
            return

        present_tick = None
        for _ in range(max(1, self.startup_read_retries)):
            present_tick = self._read_present_position(log_errors=False)
            if present_tick is not None:
                break

        if present_tick is None:
            self.get_logger().error(
                "Could not read Present Position from the Dynamixel after startup."
            )
            return

        self.last_feedback_tick = present_tick
        self.current_rad = self._tick_to_rad(present_tick)
        self.target_rad = self.current_rad
        self.commanded_rad = self.current_rad
        self.hw_ready = True

        hw_error = self._read1(ADDR_HARDWARE_ERROR_STATUS, log_errors=False)
        self.get_logger().info(
            "Dynamixel connected | "
            f"id={self.dxl_id} port={self.device_name} baud={self.baud_rate} "
            f"present={present_tick} rad={self.current_rad:.3f} hw_error={hw_error}"
        )

    def _check_result(self, comm_result, dxl_error, action: str, log_errors: bool = True):
        if comm_result != self.comm_success:
            if log_errors:
                reason = self.packet_handler.getTxRxResult(comm_result)
                self.get_logger().error(f"{action} failed: {reason}")
            return False

        if dxl_error != 0:
            if log_errors:
                reason = self.packet_handler.getRxPacketError(dxl_error)
                self.get_logger().error(f"{action} returned device error: {reason}")
            return False

        return True

    def _write1(self, addr: int, value: int, log_errors: bool = True) -> bool:
        if self.port_handler is None or self.packet_handler is None:
            return False
        comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, addr, int(value)
        )
        return self._check_result(
            comm_result, dxl_error, f"write1 addr={addr} value={value}", log_errors
        )

    def _write4(self, addr: int, value: int, log_errors: bool = True) -> bool:
        if self.port_handler is None or self.packet_handler is None:
            return False
        comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.dxl_id, addr, int(value)
        )
        return self._check_result(
            comm_result, dxl_error, f"write4 addr={addr} value={value}", log_errors
        )

    def _read1(self, addr: int, log_errors: bool = True) -> Optional[int]:
        if self.port_handler is None or self.packet_handler is None:
            return None
        value, comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.dxl_id, addr
        )
        if not self._check_result(
            comm_result, dxl_error, f"read1 addr={addr}", log_errors
        ):
            return None
        return int(value)

    def _read_present_position(self, log_errors: bool = True) -> Optional[int]:
        if self.port_handler is None or self.packet_handler is None:
            return None
        value, comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION
        )
        if not self._check_result(
            comm_result, dxl_error, "read Present Position", log_errors
        ):
            return None
        return int(value)

    def destroy_node(self):
        if self.port_handler is not None and self.packet_handler is not None:
            if self.park_on_shutdown and self.hw_ready:
                park_tick = self._rad_to_tick(self.shutdown_position)
                self._write4(ADDR_GOAL_POSITION, park_tick, log_errors=False)
            self._write1(ADDR_TORQUE_ENABLE, TORQUE_DISABLE, log_errors=False)
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
