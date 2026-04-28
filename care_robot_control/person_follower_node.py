"""
person_follower_node.py
=======================
Care robot person-following controller.

Architecture
------------
Inputs  : /person_detection  (care_robot_msgs/PersonDetection)
            - distance  [m]      : depth-estimated distance to person
            - angle     [rad]    : angle in neck_yaw_link frame (+left / -right)
            - detected  [bool]   : whether person is visible

Outputs : /cmd_vel             (geometry_msgs/Twist)  → Arduino differential drive
          /neck_yaw_target     (std_msgs/Float64)     → neck_controller_node [rad]
          /robot_state         (std_msgs/String)       → state for debug/UI

Control loops (all run at 50 Hz)
---------------------------------
Loop 1 – Distance (longitudinal)
    e_r = r - r_d
    v   = clip(Kp_v * e_r + Kd_v * ė_r, -v_max, v_max)
    Deadband: |e_r| < tol → v = 0

Loop 2 – Heading (angular)
    φ is the angle to person in neck_yaw frame.
    The body must rotate to align: ω = clip(Kp_w * φ + Kd_w * φ̇, -ω_max, ω_max)
    Neck yaw contributes additional soft compensation.

Loop 3 – Neck yaw (position control)
    θ_neck_target = φ   (published to neck_controller_node)
    When body is aligned (φ ≈ 0), neck returns to 0 with τ time-constant.

State machine
-------------
FOLLOWING  : person detected, controlling normally
SEARCHING  : person lost → neck scans ±FOV/2 at fixed angular rate
IDLE       : search timeout → stop and wait

URDF-derived parameters (care_robot.urdf.xacro)
------------------------------------------------
wheel_radius  = 0.0325  m   (65mm dia)
wheel_base    = 0.2115  m   (left_wheel_y * 2)
camera_fov_h  = 2.094   rad (~120°, ArduCam IMX708 wide)

Motor limits (JGB37-520 @ 12V, 250 RPM no-load)
------------------------------------------------
ω_wheel_max = 250 * 2π / 60 = 26.18 rad/s
v_max       = ω_wheel_max * r = 26.18 * 0.0325 ≈ 0.85 m/s
(conservative operational limit applied below)

XC330-T181 neck limits
-----------------------
max speed = 113 RPM @ 12V → ω_neck_max ≈ 11.83 rad/s (plenty for this task)
position range: 0~4095 = 0~360°, but joint limit applied ±60° (1.047 rad)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String

# Custom message — defined in care_robot_msgs package (see msg/PersonDetection.msg)
# If not yet built, temporarily use Float64MultiArray (see NOTES at bottom)
try:
    from care_robot_msgs.msg import PersonDetection
    USE_CUSTOM_MSG = True
except ImportError:
    from std_msgs.msg import Float64MultiArray
    USE_CUSTOM_MSG = False


# ─────────────────────────────────────────────
# Parameters (override via ROS2 parameters)
# ─────────────────────────────────────────────
DEFAULT_PARAMS = {
    # Geometry (from URDF)
    "wheel_radius":     0.0325,    # [m]
    "wheel_base":       0.2115,    # [m]

    # Target following distance
    "target_distance":  1.0,       # [m]
    "distance_tol":     0.10,      # [m]  deadband ±0.1 m (spec requirement)

    # PD gains — distance loop
    "Kp_v":             1.5,       # [m/s per m error]
    "Kd_v":             0.15,

    # PD gains — heading loop
    "Kp_w":             1.2,       # [rad/s per rad error]
    "Kd_w":             0.05,

    # Velocity limits
    "v_max":            0.8,       # [m/s]  conservative for indoor care robot
    "omega_max":        1.0,       # [rad/s]

    # Neck yaw
    "neck_limit":       1.047,     # [rad] ±60° hard limit
    "neck_return_gain": 0.3,       # gain for returning neck to 0 when body aligned

    # Camera FOV (ArduCam IMX708 wide, horizontal)
    "camera_fov_h":     2.094,     # [rad] ~120°

    # State machine
    "search_speed":     0.4,       # [rad/s] neck scan speed during SEARCHING
    "search_timeout":   5.0,       # [s] → IDLE after this

    # Control loop
    "control_hz":       50.0,
    "vision_timeout":   0.5,       # [s] no detection → SEARCHING
}


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower_node")

        # ── Declare & get parameters ──────────────────────────────────────
        for name, default in DEFAULT_PARAMS.items():
            self.declare_parameter(name, default)

        def p(name):
            return self.get_parameter(name).value

        self.r_d       = p("target_distance")
        self.tol       = p("distance_tol")
        self.Kp_v      = p("Kp_v")
        self.Kd_v      = p("Kd_v")
        self.Kp_w      = p("Kp_w")
        self.Kd_w      = p("Kd_w")
        self.v_max     = p("v_max")
        self.w_max     = p("omega_max")
        self.neck_lim  = p("neck_limit")
        self.neck_kret = p("neck_return_gain")
        self.fov_h     = p("camera_fov_h")
        self.search_spd = p("search_speed")
        self.search_to  = p("search_timeout")
        self.vis_to     = p("vision_timeout")
        dt_ctrl         = 1.0 / p("control_hz")

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
        if USE_CUSTOM_MSG:
            self.sub_detect = self.create_subscription(
                PersonDetection,
                "/person_detection",
                self._detection_cb,
                sensor_qos,
            )
        else:
            # Fallback: Float64MultiArray [distance, angle, detected(0/1)]
            self.get_logger().warn(
                "care_robot_msgs not found — using Float64MultiArray on /person_detection_raw"
            )
            self.sub_detect = self.create_subscription(
                Float64MultiArray,
                "/person_detection_raw",
                self._detection_raw_cb,
                sensor_qos,
            )

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_cmdvel = self.create_publisher(Twist, "/cmd_vel", reliable_qos)
        self.pub_neck   = self.create_publisher(Float64, "/neck_yaw_target", reliable_qos)
        self.pub_state  = self.create_publisher(String, "/robot_state", reliable_qos)

        # ── State ─────────────────────────────────────────────────────────
        self.distance  = None   # latest from vision [m]
        self.angle     = None   # latest from vision [rad], neck_yaw frame
        self.detected  = False

        self.prev_e_r  = 0.0   # previous distance error (for derivative)
        self.prev_phi  = 0.0   # previous angle error
        self.neck_pos  = 0.0   # current neck yaw command [rad]

        # FSM
        self.state             = "IDLE"  # IDLE | FOLLOWING | SEARCHING
        self.last_detect_time  = None
        self.search_direction  = 1.0    # +1 or -1 for sweep direction
        self.search_elapsed    = 0.0

        self.dt = dt_ctrl

        # ── Control timer ─────────────────────────────────────────────────
        self.timer = self.create_timer(dt_ctrl, self._control_loop)

        self.get_logger().info(
            f"PersonFollowerNode started | r_d={self.r_d}m | "
            f"Kp_v={self.Kp_v} Kp_w={self.Kp_w}"
        )

    # ─────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────
    def _detection_cb(self, msg: "PersonDetection"):
        self.distance = float(msg.distance)
        self.angle    = float(msg.angle)
        self.detected = bool(msg.detected)
        if self.detected:
            self.last_detect_time = self.get_clock().now()

    def _detection_raw_cb(self, msg):
        """Fallback for Float64MultiArray [distance, angle, detected]."""
        d = msg.data
        if len(d) >= 3:
            self.distance = float(d[0])
            self.angle    = float(d[1])
            self.detected = bool(d[2] > 0.5)
            if self.detected:
                self.last_detect_time = self.get_clock().now()

    # ─────────────────────────────────────────────
    # FSM transition
    # ─────────────────────────────────────────────
    def _update_state(self):
        now = self.get_clock().now()

        if self.detected:
            self.state          = "FOLLOWING"
            self.search_elapsed = 0.0
        else:
            if self.last_detect_time is None:
                self.state = "IDLE"
                return

            elapsed = (now - self.last_detect_time).nanoseconds * 1e-9

            if elapsed > self.vis_to and self.state == "FOLLOWING":
                self.state = "SEARCHING"
                self.get_logger().info("Person lost → SEARCHING")

            if self.state == "SEARCHING":
                self.search_elapsed += self.dt
                if self.search_elapsed > self.search_to:
                    self.state = "IDLE"
                    self.get_logger().info("Search timeout → IDLE")

    # ─────────────────────────────────────────────
    # Main control loop (50 Hz)
    # ─────────────────────────────────────────────
    def _control_loop(self):
        self._update_state()

        v     = 0.0
        omega = 0.0
        neck  = 0.0

        if self.state == "FOLLOWING":
            v, omega, neck = self._compute_following()

        elif self.state == "SEARCHING":
            v, omega, neck = self._compute_searching()

        else:  # IDLE
            pass  # zero output

        self._publish(v, omega, neck)
        self._publish_state()

    # ─────────────────────────────────────────────
    # Control: FOLLOWING
    # ─────────────────────────────────────────────
    def _compute_following(self):
        r   = self.distance
        phi = self.angle

        # ── Distance PD ──────────────────────────────────
        e_r  = r - self.r_d
        de_r = (e_r - self.prev_e_r) / self.dt
        self.prev_e_r = e_r

        # Deadband: within tolerance → no longitudinal motion
        if abs(e_r) < self.tol:
            v = 0.0
        else:
            v = self.Kp_v * e_r + self.Kd_v * de_r
            v = max(-self.v_max, min(self.v_max, v))

        # ── Heading PD ───────────────────────────────────
        # phi is defined in neck_yaw frame.
        # Body yaw rate to minimize phi (align robot to face person).
        # Note: neck absorbs small phi; body rotates for large phi.
        dphi = (phi - self.prev_phi) / self.dt
        self.prev_phi = phi

        omega = self.Kp_w * phi + self.Kd_w * dphi
        omega = max(-self.w_max, min(self.w_max, omega))

        # ── Neck yaw ─────────────────────────────────────
        # Direct position tracking of phi.
        # When body is well-aligned (|omega| small), return neck toward 0.
        neck = phi
        neck = max(-self.neck_lim, min(self.neck_lim, neck))

        # Soft neck-centering when body is aligned
        if abs(phi) < 0.087:  # < 5° → let body handle it, center neck
            neck = self.neck_pos + self.neck_kret * (0.0 - self.neck_pos) * self.dt

        self.neck_pos = neck
        return v, omega, neck

    # ─────────────────────────────────────────────
    # Control: SEARCHING
    # ─────────────────────────────────────────────
    def _compute_searching(self):
        """
        Oscillate neck ±FOV/2 to scan for person.
        If neck hits limit, reverse direction.
        Body stays still (v=0, omega=0).
        """
        v     = 0.0
        omega = 0.0

        # Sweep neck
        self.neck_pos += self.search_direction * self.search_spd * self.dt

        if self.neck_pos >= self.neck_lim:
            self.neck_pos     = self.neck_lim
            self.search_direction = -1.0
        elif self.neck_pos <= -self.neck_lim:
            self.neck_pos     = -self.neck_lim
            self.search_direction = 1.0

        return v, omega, self.neck_pos

    # ─────────────────────────────────────────────
    # Publishers
    # ─────────────────────────────────────────────
    def _publish(self, v: float, omega: float, neck: float):
        twist = Twist()
        twist.linear.x  = v
        twist.angular.z = omega
        self.pub_cmdvel.publish(twist)

        neck_msg = Float64()
        neck_msg.data = neck
        self.pub_neck.publish(neck_msg)

    def _publish_state(self):
        msg = String()
        r_str = f"{self.distance:.2f}" if self.distance else "N/A"
        phi_str = f"{math.degrees(self.angle):.1f}°" if self.angle else "N/A"
        msg.data = (
            f"state={self.state} | r={r_str}m | φ={phi_str} | "
            f"neck={math.degrees(self.neck_pos):.1f}°"
        )
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
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
# care_robot_msgs/PersonDetection.msg:
#   float64 distance   # [m]
#   float64 angle      # [rad]  neck_yaw frame, +CCW (left positive)
#   bool    detected
#
# If using fallback Float64MultiArray, publish: [distance, angle, 1.0/0.0]
#
# Tuning guide:
#   Kp_v: start at 0.4, increase until oscillation, then back off 20%
#   Kp_w: start at 0.8; too high → body oscillates side to side
#   Kd_*: add only if P-control shows overshoot; start at ~0.1*Kp
#
# Discretization: All derivatives are backward Euler (simplest, sufficient at 50Hz).
# For production, consider Tustin (bilinear) for the derivative term.
# ─────────────────────────────────────────────────────────────────────────────