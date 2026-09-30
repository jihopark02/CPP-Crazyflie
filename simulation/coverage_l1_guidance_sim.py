#!/usr/bin/env python3
"""
Gazebo 시뮬레이션 — 커버리지 경로 L1 유도 + 실시간 matplotlib 시각화
/cmd_vel 발행 -> control_services가 이착륙/고도유지 처리
U턴 근처에서 path_idx가 뒤로 튀지 않도록, 항상 전진(단조증가)만 하게 제한
"""
import csv
import math
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

ROBOT_NS = 'crazyflie'
PATH_CSV = '/home/won/robotics/flight_logs/l1_tests/coverage_path.csv'
LOG_CSV = '/home/won/robotics/flight_logs/l1_tests/coverage_flight_log.csv'
ORIGIN_X = 0.0
ORIGIN_Y = 0.0
V = 0.08
L1 = 0.15
GOAL_RADIUS = 0.12
CONTROL_HZ = 10.0
HOVER_HEIGHT = 0.5
FORWARD_SEARCH_DIST_M = 0.35   # 앞으로만, 이 거리(m) 안에서만 "가장 가까운 점" 탐색
DEBUG = False


def load_path(csv_path, origin_x, origin_y):
    pts = []
    dists = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            pts.append((origin_x + float(row['x']), origin_y + float(row['y'])))
            dists.append(float(row['dist']) if row.get('dist') not in (None, '') else None)
    if any(d is None for d in dists):
        dists = [0.0]
        for i in range(1, len(pts)):
            dists.append(dists[-1] + math.hypot(pts[i][0]-pts[i-1][0], pts[i][1]-pts[i-1][1]))
    return pts, dists


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_yaw(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class CoverageGuidanceSim(Node):
    def __init__(self):
        super().__init__('coverage_guidance_sim_live')
        self.path, self.dist = load_path(PATH_CSV, ORIGIN_X, ORIGIN_Y)
        self.n = len(self.path)
        self.path_idx = 0
        self.pos = None
        self.z = 0.0
        self.yaw = 0.0
        self.airborne = False
        self.done = False
        self.t0 = time.time()
        self.last_target = None
        self.trail_x = []
        self.trail_y = []

        self.get_logger().info(
            f'경로 로드: {self.n}개 점, 총 길이 {self.dist[-1]:.2f}m, 원점({ORIGIN_X:.2f},{ORIGIN_Y:.2f})')

        self.log_file = open(LOG_CSV, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['t', 'x', 'y', 'z', 'yaw_deg', 'path_idx', 'target_x', 'target_y', 'eta_deg'])

        self.create_subscription(Odometry, f'/{ROBOT_NS}/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_cb(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.z = msg.pose.pose.position.z
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.airborne:
            self.trail_x.append(self.pos[0])
            self.trail_y.append(self.pos[1])

    def update_nearest_idx(self):
        # 뒤로는 절대 안 감(단조증가). 앞으로도 FORWARD_SEARCH_DIST_M(m) 안에서만 탐색
        # -> U턴에서 경로가 스스로 접혀도 뒤쪽(이미 지나온 반대편)으로 안 튐
        cur_dist = self.dist[self.path_idx]
        hi = cur_dist + FORWARD_SEARCH_DIST_M

        end = self.path_idx
        while end < self.n - 1 and self.dist[end + 1] <= hi:
            end += 1

        best_dist = float('inf')
        best_i = self.path_idx
        for i in range(self.path_idx, end + 1):
            px, py = self.path[i]
            d = math.hypot(px - self.pos[0], py - self.pos[1])
            if d < best_dist:
                best_dist = d
                best_i = i
        self.path_idx = best_i  # best_i >= 이전 path_idx 보장됨 (단조증가)

    def find_l1_point(self):
        self.update_nearest_idx()
        for i in range(self.path_idx, self.n):
            px, py = self.path[i]
            if math.hypot(px - self.pos[0], py - self.pos[1]) >= L1:
                return px, py
        return self.path[-1]

    def finish(self, reason):
        self.done = True
        self.publish_vel(0.0, 0.0, 0.0)
        self.log_file.close()
        self.get_logger().info(f'{reason} — 로그 저장됨: {LOG_CSV}')

    def control_loop(self):
        if self.pos is None or self.done:
            return

        if not self.airborne:
            if self.z < HOVER_HEIGHT:
                self.publish_vel(0.0, 0.0, 0.5)
                return
            else:
                self.airborne = True

        gx, gy = self.path[-1]
        if (self.path_idx >= self.n - 5 and
                math.hypot(self.pos[0] - gx, self.pos[1] - gy) < GOAL_RADIUS):
            self.finish('커버리지 경로 완주')
            return

        ref_x, ref_y = self.find_l1_point()
        self.last_target = (ref_x, ref_y)
        bearing = math.atan2(ref_y - self.pos[1], ref_x - self.pos[0])
        eta = wrap_to_pi(bearing - self.yaw)

        a_s = 2 * V ** 2 / L1 * math.sin(eta)
        psi_dot = a_s / V

        t = time.time() - self.t0
        self.log_writer.writerow([f'{t:.2f}', f'{self.pos[0]:.4f}', f'{self.pos[1]:.4f}',
                                   f'{self.z:.4f}', f'{math.degrees(self.yaw):.2f}',
                                   self.path_idx, f'{ref_x:.4f}', f'{ref_y:.4f}',
                                   f'{math.degrees(eta):.2f}'])

        if DEBUG:
            self.get_logger().info(
                f'pos=({self.pos[0]:.2f},{self.pos[1]:.2f}) idx={self.path_idx}/{self.n} '
                f'target=({ref_x:.2f},{ref_y:.2f}) eta={math.degrees(eta):.1f}')

        self.publish_vel(V, psi_dot, 0.0)

    def publish_vel(self, vx, wz, vz=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.z = vz
        msg.angular.z = wz
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CoverageGuidanceSim()

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 7))
    path_x = [p[0] for p in node.path]
    path_y = [p[1] for p in node.path]
    ax.plot(path_x, path_y, 'b-', linewidth=1.0, alpha=0.5, label='planned path')
    trail_line, = ax.plot([], [], '-', color='gray', linewidth=1.5, label='flown trail')
    drone_dot, = ax.plot([], [], 'ko', markersize=10, label='drone')
    target_dot, = ax.plot([], [], 'ro', markersize=8, label='L1 target')
    ax.set_xlim(min(path_x) - 0.3, max(path_x) + 0.3)
    ax.set_ylim(min(path_y) - 0.3, max(path_y) + 0.3)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right')
    ax.set_title('Coverage path — live')

    last_control_time = time.time()
    control_period = 1.0 / CONTROL_HZ

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            now = time.time()
            if now - last_control_time >= control_period:
                node.control_loop()
                last_control_time = now

            if node.pos is not None:
                drone_dot.set_data([node.pos[0]], [node.pos[1]])
            if node.last_target is not None:
                target_dot.set_data([node.last_target[0]], [node.last_target[1]])
            if node.trail_x:
                trail_line.set_data(node.trail_x, node.trail_y)

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)

            if node.done:
                break
    except KeyboardInterrupt:
        if not node.log_file.closed:
            node.log_file.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()
