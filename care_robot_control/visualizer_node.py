"""
visualizer_node.py  (fixed)
- 시간 기준: time.time() 사용 (rclpy clock 버그 회피)
- 맵: 로봇/사람 절대 좌표 표시 + 자동 범위 조정
- 그래프: 실시간 데이터 누적
"""

import math
import time
import threading
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64, String
from geometry_msgs.msg import Twist

matplotlib.use("TkAgg")

HISTORY_LEN = 300
DT_VIZ = 0.1   # 10Hz refresh

ROBOT_L = 0.25
ROBOT_W = 0.20


class VisualizerNode(Node):
    def __init__(self):
        super().__init__("visualizer_node")

        self.create_subscription(Float64MultiArray, "/sim_robot_pose",  self._robot_cb,  10)
        self.create_subscription(Float64MultiArray, "/sim_person_pose", self._person_cb, 10)
        self.create_subscription(Float64MultiArray, "/person_detection_raw", self._detect_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmdvel_cb, 10)
        self.create_subscription(String, "/robot_state", self._state_cb, 10)

        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_th  = 0.0
        self.neck_yaw  = 0.0
        self.person_x  = 1.5
        self.person_y  = 0.0
        self.distance  = 1.5
        self.angle     = 0.0
        self.detected  = False
        self.v_cmd     = 0.0
        self.w_cmd     = 0.0
        self.state_str = "IDLE"

        self.robot_trail  = deque(maxlen=200)
        self.person_trail = deque(maxlen=200)

        self.t_hist  = deque(maxlen=HISTORY_LEN)
        self.er_hist = deque(maxlen=HISTORY_LEN)
        self.v_hist  = deque(maxlen=HISTORY_LEN)
        self.w_hist  = deque(maxlen=HISTORY_LEN)
        self.ny_hist = deque(maxlen=HISTORY_LEN)

        self.t0 = time.time()   # wall clock 기준

    def _robot_cb(self, msg):
        d = msg.data
        self.robot_x  = d[0]; self.robot_y  = d[1]
        self.robot_th = d[2]; self.neck_yaw = d[3]
        self.robot_trail.append((d[0], d[1]))

    def _person_cb(self, msg):
        d = msg.data
        self.person_x = d[0]; self.person_y = d[1]
        self.person_trail.append((d[0], d[1]))

    def _detect_cb(self, msg):
        d = msg.data
        self.distance = d[0]
        self.angle    = d[1]
        self.detected = d[2] > 0.5

        t = time.time() - self.t0   # wall clock 기준 시간
        self.t_hist.append(t)
        self.er_hist.append(self.distance - 1.0)
        self.v_hist.append(self.v_cmd)
        self.w_hist.append(self.w_cmd)
        self.ny_hist.append(math.degrees(self.neck_yaw))

    def _cmdvel_cb(self, msg):
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z

    def _state_cb(self, msg):
        self.state_str = msg.data


def draw_robot(ax, rx, ry, rth, neck_yaw):
    c, s = math.cos(rth), math.sin(rth)
    R = np.array([[c, -s], [s, c]])
    corners = np.array([
        [+ROBOT_L/2, +ROBOT_W/2],
        [+ROBOT_L/2, -ROBOT_W/2],
        [-ROBOT_L/2, -ROBOT_W/2],
        [-ROBOT_L/2, +ROBOT_W/2],
    ])
    cw = (R @ corners.T).T + np.array([rx, ry])
    body = plt.Polygon(cw, closed=True, facecolor="#0f3460", edgecolor="#e94560", lw=1.5)
    ax.add_patch(body)

    # 전진 방향 화살표
    ax.annotate("", xy=(rx + 0.2*c, ry + 0.2*s), xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="#e94560", lw=2))

    # 목 방향
    nth = rth + neck_yaw
    ax.annotate("", xy=(rx + 0.35*math.cos(nth), ry + 0.35*math.sin(nth)),
                xytext=(rx, ry),
                arrowprops=dict(arrowstyle="->", color="#00b4d8", lw=1.5))

    # FOV 선 ±60°
    for sign in [+1, -1]:
        a = nth + sign * math.radians(60)
        ax.plot([rx, rx + 1.0*math.cos(a)],
                [ry, ry + 1.0*math.sin(a)],
                color="#00b4d8", lw=0.7, alpha=0.5)


def run_visualizer(vis: VisualizerNode):
    fig = plt.figure(figsize=(14, 7))
    fig.patch.set_facecolor("#1a1a2e")

    ax_map = fig.add_subplot(1, 2, 1)
    gs = fig.add_gridspec(3, 2, left=0.55, right=0.97, hspace=0.55, top=0.93, bottom=0.08)
    ax_er  = fig.add_subplot(gs[0, :])
    ax_vel = fig.add_subplot(gs[1, :])
    ax_ny  = fig.add_subplot(gs[2, :])

    for ax in [ax_map, ax_er, ax_vel, ax_ny]:
        ax.set_facecolor("#16213e")
        ax.tick_params(colors="gray", labelsize=7)
        ax.grid(True, color="#333", lw=0.5)
        for sp in ax.spines.values():
            sp.set_edgecolor("#444")

    plt.ion()
    plt.show()

    while rclpy.ok():
        # ── Map ──────────────────────────────────────────────────────────
        ax_map.cla()
        ax_map.set_facecolor("#16213e")
        ax_map.set_aspect("equal")
        ax_map.tick_params(colors="gray", labelsize=7)
        ax_map.grid(True, color="#333", lw=0.5)

        # 자동 범위: 로봇 중심 ±3m
        cx, cy = vis.robot_x, vis.robot_y
        margin = 3.0
        ax_map.set_xlim(cx - margin, cx + margin)
        ax_map.set_ylim(cy - margin, cy + margin)

        # Trail
        if len(vis.robot_trail) > 1:
            tx, ty = zip(*vis.robot_trail)
            ax_map.plot(tx, ty, color="#e94560", lw=0.8, alpha=0.5)
        if len(vis.person_trail) > 1:
            px, py = zip(*vis.person_trail)
            ax_map.plot(px, py, color="#06d6a0", lw=0.8, alpha=0.5)

        # Robot
        draw_robot(ax_map, vis.robot_x, vis.robot_y, vis.robot_th, vis.neck_yaw)

        # Person
        p_color = "#06d6a0" if vis.detected else "#ff6b6b"
        ax_map.plot(vis.person_x, vis.person_y, "o", color=p_color, ms=12, zorder=5)
        ax_map.plot(vis.person_x, vis.person_y, "o", color=p_color, ms=22, alpha=0.2, zorder=4)

        # 목표 거리 원 (1m)
        circle = plt.Circle((vis.robot_x, vis.robot_y), 1.0,
                             color="#ffaa00", fill=False, lw=0.8, ls="--", alpha=0.5)
        ax_map.add_patch(circle)

        # 원점 표시
        ax_map.plot(0, 0, "+", color="#555", ms=8)

        state_color = {"FOLLOWING": "#06d6a0", "SEARCHING": "#ffaa00"}.get(
            vis.state_str.split()[0] if vis.state_str else "IDLE", "#aaaaaa"
        )
        detected_str = "✓" if vis.detected else "✗"
        ax_map.set_title(
            f"[{vis.state_str.split('=')[1].split('|')[0].strip() if '=' in vis.state_str else vis.state_str}]  "
            f"r={vis.distance:.2f}m  φ={math.degrees(vis.angle):.1f}°  det={detected_str}",
            color=state_color, fontsize=9
        )

        legend = [
            mpatches.Patch(color="#e94560", label="Robot"),
            mpatches.Patch(color=p_color,   label=f"Person {detected_str}"),
            mpatches.Patch(color="#00b4d8", label="Camera FOV"),
        ]
        ax_map.legend(handles=legend, loc="upper right",
                      facecolor="#1a1a2e", labelcolor="white", fontsize=7)

        # ── Time-series ───────────────────────────────────────────────────
        if len(vis.t_hist) > 2:
            t  = list(vis.t_hist)
            er = list(vis.er_hist)
            v  = list(vis.v_hist)
            w  = list(vis.w_hist)
            ny = list(vis.ny_hist)

            # Distance error
            ax_er.cla(); ax_er.set_facecolor("#16213e")
            ax_er.tick_params(colors="gray", labelsize=7)
            ax_er.grid(True, color="#333", lw=0.5)
            ax_er.plot(t, er, color="#e94560", lw=1.2)
            ax_er.axhline( 0.1, color="#ffaa00", ls="--", lw=0.8, alpha=0.7)
            ax_er.axhline(-0.1, color="#ffaa00", ls="--", lw=0.8, alpha=0.7)
            ax_er.axhline( 0.0, color="#555",    lw=0.5)
            ax_er.set_ylabel("e_r [m]", color="gray", fontsize=7)
            ax_er.set_title("Distance Error  (목표 1.0m, 허용 ±0.1m)", color="white", fontsize=8)

            # Velocity
            ax_vel.cla(); ax_vel.set_facecolor("#16213e")
            ax_vel.tick_params(colors="gray", labelsize=7)
            ax_vel.grid(True, color="#333", lw=0.5)
            ax_vel.plot(t, v, color="#00b4d8", lw=1.2, label="v [m/s]")
            ax_vel.plot(t, w, color="#f77f00", lw=1.2, label="ω [rad/s]")
            ax_vel.axhline(0.0, color="#555", lw=0.5)
            ax_vel.set_title("Velocity Commands", color="white", fontsize=8)
            ax_vel.legend(facecolor="#1a1a2e", labelcolor="white", fontsize=7, loc="upper right")

            # Neck yaw
            ax_ny.cla(); ax_ny.set_facecolor("#16213e")
            ax_ny.tick_params(colors="gray", labelsize=7)
            ax_ny.grid(True, color="#333", lw=0.5)
            ax_ny.plot(t, ny, color="#06d6a0", lw=1.2)
            ax_ny.axhline( 60, color="#ff4444", ls="--", lw=0.8, alpha=0.7)
            ax_ny.axhline(-60, color="#ff4444", ls="--", lw=0.8, alpha=0.7)
            ax_ny.axhline(  0, color="#555", lw=0.5)
            ax_ny.set_title("Neck Yaw [°]  (limit ±60°)", color="white", fontsize=8)
            ax_ny.set_xlabel("time [s]", color="gray", fontsize=7)

        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(DT_VIZ)


def main():
    rclpy.init()
    vis = VisualizerNode()
    thread = threading.Thread(target=rclpy.spin, args=(vis,), daemon=True)
    thread.start()
    try:
        run_visualizer(vis)
    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()