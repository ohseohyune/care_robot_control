import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from senior_msg.msg import ImuMsg


class EbimuNode(Node):
    def __init__(self):
        super().__init__("ebimu_node")

        self.declare_parameter("port_name", "/dev/ttyUSB-EBIMU")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("loop_rate", 100)

        self.port_name = self.get_parameter("port_name").value
        self.baudrate = self.get_parameter("baudrate").value
        loop_rate = self.get_parameter("loop_rate").value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(ImuMsg, "Imu", qos)

        self._q = [0.0, 0.0, 0.0, 0.0]  # q0..q3 as received
        self._lock = threading.Lock()
        self._serial = None

        self._open_serial()

        period = 1.0 / max(loop_rate, 1)
        self.create_timer(period, self._publish)

        read_thread = threading.Thread(target=self._read_loop, daemon=True)
        read_thread.start()

        self.get_logger().info(
            f"EbimuNode ready: port={self.port_name} baud={self.baudrate}"
        )

    def _open_serial(self):
        try:
            import serial
            self._serial = serial.Serial(self.port_name, self.baudrate, timeout=1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit(1)

    def _read_loop(self):
        """시리얼에서 패킷을 읽어 quaternion을 갱신하는 백그라운드 스레드."""
        buf = b""
        while rclpy.ok():
            try:
                chunk = self._serial.read(self._serial.in_waiting or 1)
            except Exception:
                break
            buf += chunk
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                self._parse(line)

    def _parse(self, raw: bytes):
        """패킷 파싱: *q0,q1,q2,q3,ar0,ar1,ar2,a0,a1,a2"""
        try:
            text = raw.decode("ascii", errors="ignore").strip()
        except Exception:
            return
        if not text.startswith("*"):
            return
        parts = text[1:].split(",")
        if len(parts) < 4:
            return
        try:
            vals = [float(p) for p in parts[:4]]
        except ValueError:
            return
        with self._lock:
            self._q = vals

    def _publish(self):
        with self._lock:
            q = self._q[:]

        # C++ 원본과 동일한 인덱스 매핑
        x = q[2]
        y = q[1]
        z = q[0]
        w = q[3]

        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_rad = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch_rad = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_rad = math.atan2(t3, t4)

        msg = ImuMsg()
        msg.roll  = float((pitch_rad * 180.0) / math.pi)
        msg.pitch = float((roll_rad  * 180.0) / math.pi)
        msg.yaw   = float(-(yaw_rad  * 180.0) / math.pi)
        self.pub.publish(msg)

    def destroy_node(self):
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EbimuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
