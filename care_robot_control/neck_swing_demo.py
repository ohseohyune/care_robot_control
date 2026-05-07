"""
neck_swing_demo.py
발표용 목 모터 왔다갔다 데모.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math


class NeckSwingDemo(Node):
    def __init__(self):
        super().__init__("neck_swing_demo")

        self.declare_parameter("amplitude", 0.5)   # 최대 각도 (rad), 약 +-28도
        self.declare_parameter("period", 3.0)       # 왕복 주기 (초)
        self.declare_parameter("publish_hz", 20.0)

        self.amplitude = self.get_parameter("amplitude").value
        self.period = self.get_parameter("period").value
        publish_hz = self.get_parameter("publish_hz").value

        self.pub = self.create_publisher(Float64, "/neck_yaw_target", 10)
        self.t = 0.0
        self.dt = 1.0 / publish_hz

        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f"NeckSwingDemo started: amplitude=+-{math.degrees(self.amplitude):.1f}deg "
            f"period={self.period}s  (Ctrl+C to stop)"
        )

    def _tick(self):
        angle = self.amplitude * math.sin(2.0 * math.pi * self.t / self.period)
        msg = Float64()
        msg.data = angle
        self.pub.publish(msg)
        self.t += self.dt


def main(args=None):
    rclpy.init(args=args)
    node = NeckSwingDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Float64()
        stop.data = 0.0
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
