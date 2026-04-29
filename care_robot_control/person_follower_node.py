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
    "distance_tol":     0.05,      # [m]  tighter deadband for lower steady-state error

    # PID gains — distance loop
    # I term eliminates steady-state error when tracking a moving person
    # (P-only: SS error = v_person / Kp_v ≈ 0.5m at 0.3 m/s)
    "Kp_v":             1.2,       # [m/s per m error]
    "Ki_v":             0.4,       # [m/s per m·s] — drives SS error → 0
    "Kd_v":             0.15,
    "i_max_v":          0.5,       # [m/s] anti-windup: Ki*integral clamped to ±i_max_v
    "distance_filter_alpha": 0.35, # LPF for noisy distance measurement (0<alpha<=1)
    "deadband_integral_decay": 0.92,  # preserve some integral action near target

    # PD gains — heading loop
    "Kp_w":             1.2,       # [rad/s per rad error]
    "Kd_w":             0.05,
    "heading_slowdown_angle": 0.70,   # [rad] start reducing forward speed for large yaw error

    # Velocity limits
    # Frail elderly walk at 0.4~0.6 m/s → robot needs to exceed that to catch up
    "v_max":            0.6,       # [m/s]  slightly above elderly walking speed
    "omega_max":        1.0,       # [rad/s]

    # Neck yaw
    "neck_limit":       1.047,     # [rad] ±60° hard limit
    "neck_return_gain": 0.3,       # gain for returning neck to 0 when body aligned

    # Camera FOV (ArduCam IMX708 wide, horizontal)
    "camera_fov_h":     2.094,     # [rad] ~120°

    # State machine — search pattern (body 120° rotate → neck ±60° scan, repeat)
    # Target: complete 360° search before person walks out of 5m detection range
    # At 0.5 m/s, person travels ~5m in 10s → keep total search time under ~20s
    "search_speed":     0.8,       # [rad/s] neck scan: ±60° in ~2.6s
    "search_omega":     0.6,       # [rad/s] body rotation: 120° in ~3.5s

    # Control loop
    "control_hz":       50.0,
    "vision_timeout":   0.3,       # [s] no detection → SEARCHING (react quickly)
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
        self.Ki_v      = p("Ki_v")
        self.Kd_v      = p("Kd_v")
        self.i_max_v   = p("i_max_v")
        self.dist_alpha = p("distance_filter_alpha")
        self.deadband_i_decay = p("deadband_integral_decay")
        self.Kp_w      = p("Kp_w")
        self.Kd_w      = p("Kd_w")
        self.heading_slow_angle = p("heading_slowdown_angle")
        self.v_max     = p("v_max")
        self.w_max     = p("omega_max")
        self.neck_lim  = p("neck_limit")
        self.neck_kret = p("neck_return_gain")
        self.fov_h     = p("camera_fov_h")
        self.search_spd   = p("search_speed")
        self.search_omega = p("search_omega")
        self.vis_to       = p("vision_timeout")
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

        self.prev_e_r   = 0.0   # previous distance error (for derivative)
        self.e_r_integ  = 0.0   # distance error integral (for I term)
        self.prev_phi   = 0.0   # previous angle error
        self.neck_pos   = 0.0   # current neck yaw command [rad]
        self.distance_filt = None

        # FSM
        self.state            = "IDLE"  # IDLE | FOLLOWING | SEARCHING
        self.last_detect_time = None
        self.last_known_angle = 0.0     # person's angle at last detection [rad]

        # SEARCHING sub-state machine
        #   Pattern: [body rotates 120°] → [neck scans ±60°]  ×2 segments = 1 loop
        #   Repeat up to 2 loops (4 segments total) then IDLE
        self.search_sub_state     = "BODY_ROTATING"  # "BODY_ROTATING" | "NECK_SCANNING"
        self.search_segment_count = 0     # completed segments (each = rotate+scan)
        self.search_body_rotated  = 0.0   # accumulated body rotation this segment [rad]
        self.search_body_dir      = 1.0   # +1=CCW / -1=CW
        self.neck_scan_dir        = 1.0   # neck sweep direction during NECK_SCANNING

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
        self.distance_filt = self._filter_distance(self.distance)
        self.angle    = float(msg.angle)
        self.detected = bool(msg.detected)
        if self.detected:
            self.last_detect_time  = self.get_clock().now()
            self.last_known_angle  = self.angle

    def _detection_raw_cb(self, msg):
        """Fallback for Float64MultiArray [distance, angle, detected]."""
        d = msg.data
        if len(d) >= 3:
            self.distance = float(d[0])
            self.distance_filt = self._filter_distance(self.distance)
            self.angle    = float(d[1])
            self.detected = bool(d[2] > 0.5)
            if self.detected:
                self.last_detect_time = self.get_clock().now()
                self.last_known_angle = self.angle

    def _filter_distance(self, distance: float) -> float:
        if self.distance_filt is None:
            return distance
        alpha = max(1e-3, min(1.0, self.dist_alpha))
        return alpha * distance + (1.0 - alpha) * self.distance_filt

    # ─────────────────────────────────────────────
    # FSM transition
    # ─────────────────────────────────────────────
    def _update_state(self):
        now = self.get_clock().now()

        if self.detected:
            if self.state != "FOLLOWING":
                self.e_r_integ = 0.0   # stale integral from search phase → discard
            self.state = "FOLLOWING"
            return

        if self.last_detect_time is None:
            self.state = "IDLE"
            return

        elapsed = (now - self.last_detect_time).nanoseconds * 1e-9

        if elapsed > self.vis_to and self.state == "FOLLOWING":
            self.state                = "SEARCHING"
            self.search_sub_state     = "BODY_ROTATING"
            self.search_segment_count = 0
            self.search_body_rotated  = 0.0
            self.neck_pos             = 0.0
            self.neck_scan_dir        = 1.0
            # rotate toward the direction person was last seen
            self.search_body_dir = 1.0 if self.last_known_angle >= 0.0 else -1.0
            self.get_logger().info(
                f"Person lost → SEARCHING "
                f"(body={'CCW' if self.search_body_dir > 0 else 'CW'})"
            )

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
        r   = self.distance_filt if self.distance_filt is not None else self.distance
        phi = self.angle

        # ── Distance PID ─────────────────────────────────
        e_r  = r - self.r_d
        de_r = (e_r - self.prev_e_r) / self.dt
        self.prev_e_r = e_r

        # Deadband: hold near the setpoint, but keep some integral memory so the
        # robot does not fully "give up" when the person keeps moving slowly.
        if abs(e_r) < self.tol:
            v = 0.0
            self.e_r_integ *= self.deadband_i_decay
        else:
            # Integrate with anti-windup clamp
            self.e_r_integ += e_r * self.dt
            integ_limit = self.i_max_v / max(self.Ki_v, 1e-6)
            self.e_r_integ = max(-integ_limit, min(integ_limit, self.e_r_integ))

            v = (self.Kp_v * e_r
                 + self.Ki_v * self.e_r_integ
                 + self.Kd_v * de_r)
            v = max(-self.v_max, min(self.v_max, v))

        # If the person is far off-center, prioritize turning before charging ahead.
        slowdown = math.cos(min(abs(phi), self.heading_slow_angle))
        slowdown = max(0.0, slowdown)
        v *= slowdown

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
        Systematic 360° search in two phases per segment:

          BODY_ROTATING : wheels spin 120° while neck returns to center.
          NECK_SCANNING : neck sweeps +60° → -60° (covers 120° FOV).

        Two segments = one loop (covers 360° together with the original FOV).
        After 2 loops (4 segments total) without detection → IDLE.
        """
        _BODY_TARGET  = 2.0 * math.pi / 3.0  # 120° per segment
        _MAX_SEGMENTS = 4                      # 2 loops × 2 segments

        v = 0.0
        omega = 0.0

        if self.search_sub_state == "BODY_ROTATING":
            omega = self.search_body_dir * self.search_omega
            self.search_body_rotated += self.search_omega * self.dt

            # Neck drifts back to center during body rotation
            self.neck_pos *= max(0.0, 1.0 - 5.0 * self.dt)

            if self.search_body_rotated >= _BODY_TARGET:
                self.search_body_rotated = 0.0
                self.neck_pos            = 0.0
                self.neck_scan_dir       = 1.0   # start scan: center → +lim → -lim
                self.search_sub_state    = "NECK_SCANNING"
                self.get_logger().info(
                    f"Body +120° done → neck scan "
                    f"(seg {self.search_segment_count + 1}/{_MAX_SEGMENTS})"
                )

        else:  # NECK_SCANNING
            self.neck_pos += self.neck_scan_dir * self.search_spd * self.dt

            if self.neck_scan_dir > 0.0 and self.neck_pos >= self.neck_lim:
                self.neck_pos      = self.neck_lim
                self.neck_scan_dir = -1.0          # reverse: sweep toward -lim

            elif self.neck_scan_dir < 0.0 and self.neck_pos <= -self.neck_lim:
                # Reached -lim → this segment is complete
                self.neck_pos = -self.neck_lim
                self.search_segment_count += 1
                self.get_logger().info(
                    f"Neck scan done — {self.search_segment_count}/{_MAX_SEGMENTS} segments"
                )

                if self.search_segment_count >= _MAX_SEGMENTS:
                    self.state    = "IDLE"
                    self.neck_pos = 0.0
                    self.get_logger().info("Search complete (2 loops) → IDLE")
                else:
                    self.search_body_rotated = 0.0
                    self.search_sub_state    = "BODY_ROTATING"

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
