"""
robot_simulator_node.py
=======================
Software-in-the-loop simulator for care robot.
Replaces physical hardware + vision module for control validation.

Simulates
---------
1. Unicycle dynamics (differential drive, continuous-time ODE, Euler integration)
2. Person trajectory (configurable: circular, linear, standing)
3. Mock vision output: computes (distance, angle) in neck_yaw frame
   from ground-truth robot & person poses + additive Gaussian noise
4. Motor dynamics: 1st-order lag on wheel velocities (τ = 0.08 s)

Publishes
---------
/person_detection_raw   Float64MultiArray  [distance, angle, detected]
/odom                   Odometry           ground-truth robot pose
/sim_person_pose        Float64MultiArray  [px, py]   (for visualization)
/sim_robot_pose         Float64MultiArray  [rx, ry, rtheta, neck_yaw]

Subscribes
----------
/cmd_vel                Twist              from person_follower_node
/neck_yaw_state         Float64            from neck_controller_node

Parameters
----------
sim_dt          : 0.005  [s]   inner integration step (200 Hz)
publish_dt      : 0.0667 [s]   vision publish rate (15 Hz)
noise_dist_std  : 0.03   [m]   distance measurement noise σ
noise_angle_std : 0.02   [rad] angle measurement noise σ
person_mode     : "circle" | "linear" | "still"
person_speed    : 0.3    [m/s]
person_radius   : 2.0    [m]   (circle mode)
motor_tau       : 0.08   [s]   1st-order lag time constant

URDF-derived geometry
---------------------
wheel_radius = 0.0325 m
wheel_base   = 0.2115 m
neck_x       = 0.1244 m  (relative to base_link, along +x)
neck_z       = 0.2358 m  (height, irrelevant for 2D sim)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray


class RobotSimulatorNode(Node):
    def __init__(self):
        super().__init__("robot_simulator_node")

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter("sim_dt",          0.005)
        self.declare_parameter("publish_dt",      1.0 / 15.0)
        self.declare_parameter("noise_dist_std",  0.03)
        self.declare_parameter("noise_angle_std", 0.02)
        self.declare_parameter("person_mode",     "circle")
        self.declare_parameter("person_speed",    0.3)
        self.declare_parameter("person_radius",   2.0)
        self.declare_parameter("motor_tau",       0.08)
        self.declare_parameter("wheel_radius",    0.0325)
        self.declare_parameter("wheel_base",      0.2115)
        self.declare_parameter("neck_offset_x",   0.1244)
        self.declare_parameter("detection_range", 5.0)    # [m] max detection range

        def p(n): return self.get_parameter(n).value

        self.sim_dt      = p("sim_dt")
        self.pub_dt      = p("publish_dt")
        self.noise_d_std = p("noise_dist_std")
        self.noise_a_std = p("noise_angle_std")
        self.person_mode = p("person_mode")
        self.person_spd  = p("person_speed")
        self.person_rad  = p("person_radius")
        self.motor_tau   = p("motor_tau")
        self.r_w         = p("wheel_radius")
        self.L           = p("wheel_base")
        self.neck_ox     = p("neck_offset_x")
        self.det_range   = p("detection_range")

        # ── QoS ──────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.sub_cmdvel = self.create_subscription(
            Twist, "/cmd_vel", self._cmdvel_cb, reliable_qos
        )
        self.sub_neck = self.create_subscription(
            Float64, "/neck_yaw_state", self._neck_cb, sensor_qos
        )

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_detect  = self.create_publisher(
            Float64MultiArray, "/person_detection_raw", sensor_qos
        )
        self.pub_odom    = self.create_publisher(Odometry, "/odom", reliable_qos)
        self.pub_person  = self.create_publisher(
            Float64MultiArray, "/sim_person_pose", sensor_qos
        )
        self.pub_robot   = self.create_publisher(
            Float64MultiArray, "/sim_robot_pose", sensor_qos
        )

        # ── Robot state  [x, y, θ]  world frame ──────────────────────────
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_th  = 0.0

        # ── Motor state (1st-order lag on v, ω) ──────────────────────────
        self.v_cmd  = 0.0   # commanded
        self.w_cmd  = 0.0
        self.v_act  = 0.0   # actual (filtered)
        self.w_act  = 0.0

        # ── Neck state ────────────────────────────────────────────────────
        self.neck_yaw = 0.0   # [rad]

        # ── Person state  [px, py, phase] ─────────────────────────────────
        # Initial position: 1.5 m ahead of robot
        self.person_x     = 1.5
        self.person_y     = 0.0
        self.person_phase = 0.0   # for circular trajectory

        # ── Timing ────────────────────────────────────────────────────────
        self._steps_per_pub = max(1, round(self.pub_dt / self.sim_dt))
        self._step_count    = 0
        self.sim_time       = 0.0

        # ── Timer ─────────────────────────────────────────────────────────
        self.timer = self.create_timer(self.sim_dt, self._sim_step)

        self.get_logger().info(
            f"RobotSimulatorNode | person_mode={self.person_mode} | "
            f"sim_dt={self.sim_dt*1000:.0f}ms | "
            f"publish_dt={self.pub_dt*1000:.0f}ms (15Hz vision)"
        )

    # ─────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────
    def _cmdvel_cb(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)

    def _neck_cb(self, msg: Float64):
        self.neck_yaw = float(msg.data)

    # ─────────────────────────────────────────────
    # Simulation step (200 Hz)
    # ─────────────────────────────────────────────
    def _sim_step(self):
        dt = self.sim_dt

        # ── Motor lag (1st-order: τ ẋ + x = u) ───────────────────────────
        alpha     = dt / (self.motor_tau + dt)   # Euler discretization
        self.v_act = self.v_act + alpha * (self.v_cmd - self.v_act)
        self.w_act = self.w_act + alpha * (self.w_cmd - self.w_act)

        # ── Unicycle kinematics ──────────────────────────────────────────
        self.robot_x  += self.v_act * math.cos(self.robot_th) * dt
        self.robot_y  += self.v_act * math.sin(self.robot_th) * dt
        self.robot_th += self.w_act * dt
        self.robot_th  = self._wrap_angle(self.robot_th)

        # ── Person trajectory ─────────────────────────────────────────────
        self._update_person(dt)

        self.sim_time += dt
        self._step_count += 1

        # ── Publish at vision rate (15 Hz) ────────────────────────────────
        if self._step_count >= self._steps_per_pub:
            self._step_count = 0
            self._publish_detection()
            self._publish_odom()
            self._publish_debug_poses()

    # ─────────────────────────────────────────────
    # Person motion models
    # ─────────────────────────────────────────────
    def _update_person(self, dt: float):
        if self.person_mode == "circle":
            # Circular trajectory centered at world origin
            omega_person = self.person_spd / self.person_rad
            self.person_phase += omega_person * dt
            self.person_x = self.person_rad * math.cos(self.person_phase)
            self.person_y = self.person_rad * math.sin(self.person_phase)

        elif self.person_mode == "linear":
            # Walk in +x direction, wrap around at 5m
            self.person_x += self.person_spd * dt
            if self.person_x > 5.0:
                self.person_x = -2.0

        elif self.person_mode == "still":
            pass  # person does not move

    # ─────────────────────────────────────────────
    # Vision: compute (distance, angle) in neck_yaw frame
    # ─────────────────────────────────────────────
    def _compute_detection(self):
        """
        Camera is mounted on neck_yaw_link.
        neck_yaw_link position in world:
            cx = robot_x + neck_ox * cos(robot_th)
            cy = robot_y + neck_ox * sin(robot_th)
        neck_yaw_link orientation: robot_th + neck_yaw

        Angle to person in neck_yaw frame:
            Δx_world = person_x - cx
            Δy_world = person_y - cy
            φ_world  = atan2(Δy, Δx)
            φ_neck   = wrap(φ_world - (robot_th + neck_yaw))
        """
        cam_th = self.robot_th + self.neck_yaw
        cx = self.robot_x + self.neck_ox * math.cos(self.robot_th)
        cy = self.robot_y + self.neck_ox * math.sin(self.robot_th)

        dx = self.person_x - cx
        dy = self.person_y - cy

        distance_true = math.hypot(dx, dy)
        angle_world   = math.atan2(dy, dx)
        angle_neck    = self._wrap_angle(angle_world - cam_th)

        # Check if within camera FOV (horizontal ±60° = ±1.047 rad)
        fov_half = 1.047
        detected = (distance_true < self.det_range) and (abs(angle_neck) < fov_half)

        # Add measurement noise
        distance_meas = distance_true + np.random.normal(0.0, self.noise_d_std)
        angle_meas    = angle_neck    + np.random.normal(0.0, self.noise_a_std)

        return distance_meas, angle_meas, detected

    # ─────────────────────────────────────────────
    # Publishers
    # ─────────────────────────────────────────────
    def _publish_detection(self):
        dist, angle, detected = self._compute_detection()

        msg = Float64MultiArray()
        # Layout: [distance, angle, detected(1.0/0.0)]
        msg.data = [dist, angle, 1.0 if detected else 0.0]
        self.pub_detect.publish(msg)

    def _publish_odom(self):
        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        odom.pose.pose.position.x = self.robot_x
        odom.pose.pose.position.y = self.robot_y

        # Quaternion from yaw (z-axis rotation)
        odom.pose.pose.orientation.z = math.sin(self.robot_th / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.robot_th / 2.0)

        odom.twist.twist.linear.x  = self.v_act
        odom.twist.twist.angular.z = self.w_act

        self.pub_odom.publish(odom)

    def _publish_debug_poses(self):
        person_msg = Float64MultiArray()
        person_msg.data = [self.person_x, self.person_y]
        self.pub_person.publish(person_msg)

        robot_msg = Float64MultiArray()
        robot_msg.data = [
            self.robot_x, self.robot_y, self.robot_th, self.neck_yaw
        ]
        self.pub_robot.publish(robot_msg)

    # ─────────────────────────────────────────────
    # Utility
    # ─────────────────────────────────────────────
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None):
    rclpy.init(args=args)
    node = RobotSimulatorNode()
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
# Motor lag model:
#   JGB37-520 has no explicit rise time spec, but τ = 80ms is a reasonable
#   conservative estimate for small DC geared motors. Tune empirically on HW.
#
# Vision noise:
#   noise_dist_std = 0.03 m  → realistic for depth camera at ~1m
#   noise_angle_std = 0.02 rad → ~1.1° angle noise, reasonable for IMX708
#
# Person mode switching at runtime (no restart needed):
#   ros2 param set /robot_simulator_node person_mode linear
#   (Not implemented as dynamic reconfigure here — add if needed)
# ─────────────────────────────────────────────────────────────────────────────