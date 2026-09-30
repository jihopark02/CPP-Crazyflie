#!/usr/bin/env python3
"""
Gazebo 시뮬레이션 -- 경로 오차 개선판 유도 (코너 감속 제외)
coverage_guidance_sim_live.py의 ROS2/시각화 하네스는 그대로 두고,
유도부만 direct_cflib_coverage_refined2.py의 PathGuidance로 교체했다.
  1. 가장 가까운 경로 '점' 대신 경로 '선분'에 현재 위치를 투영
  2. 경로 접선 속도 + 횡오차 보정 속도를 동시에 명령 (linear.y로 직접 횡이동)
  3. 코너 자동감속은 포함하지 않음 -- 순항속도 V 항상 고정
"""
import bisect
import csv
import math
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

ROBOT_NS = 'crazyflie'
PATH_CSV = sys.argv[1] if len(sys.argv) > 1 else '/home/won/robotics/flight_logs/l1_tests/coverage_path.csv'
LOG_CSV = sys.argv[2] if len(sys.argv) > 2 else '/home/won/robotics/flight_logs/l1_tests/coverage_flight_log_refined_sim.csv'
PLOT_PNG = sys.argv[3] if len(sys.argv) > 3 else None
# 시각화용 장애물 표시 (x,y,size) -- 실제 충돌은 Gazebo world의 obstacle_* 모델이 담당, 이건 그림용
OBSTACLES = [tuple(float(v) for v in o.split(',')) for o in sys.argv[4:]] if len(sys.argv) > 4 else []
ORIGIN_X = 0.0
ORIGIN_Y = 0.0
V = 0.08
L1 = 0.15
GOAL_RADIUS = 0.12
CONTROL_HZ = 10.0
HOVER_HEIGHT = 0.5
FORWARD_SEARCH_DIST_M = 0.35

# 경로 오차 보정 튜닝값 (direct_cflib_coverage_refined2.py와 동일)
CROSS_TRACK_KP = 1.3       # [1/s]
MAX_CROSS_SPEED = 0.10     # [m/s]
MAX_YAW_RATE_DEG = 90.0    # [deg/s]
YAW_KP = 1.8               # [1/s]
DEBUG = False


def clamp(value, low, high):
    return max(low, min(high, value))


def load_path(csv_path, origin_x, origin_y):
    pts, dists = [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
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


class PathGuidance:
    """단조 증가하는 경로 진행도와 선분 투영 기반 유도 (코너 감속 없음)."""

    def __init__(self, path, dist):
        self.path = path
        self.dist = dist
        self.n = len(path)
        self.path_idx = 0
        self.progress_s = dist[0]

    @staticmethod
    def _project_segment(x, y, ax, ay, bx, by):
        abx, aby = bx - ax, by - ay
        length_sq = abx * abx + aby * aby
        if length_sq < 1e-12:
            return ax, ay, 0.0, math.hypot(x - ax, y - ay)
        u = clamp(((x - ax) * abx + (y - ay) * aby) / length_sq, 0.0, 1.0)
        qx, qy = ax + u * abx, ay + u * aby
        return qx, qy, u, math.hypot(x - qx, y - qy)

    def _point_at_s(self, s):
        s = clamp(s, self.dist[0], self.dist[-1])
        i = clamp(bisect.bisect_right(self.dist, s) - 1, 0, self.n - 2)
        ds = self.dist[i + 1] - self.dist[i]
        u = 0.0 if ds <= 1e-12 else (s - self.dist[i]) / ds
        ax, ay = self.path[i]
        bx, by = self.path[i + 1]
        return ax + u * (bx - ax), ay + u * (by - ay), i

    def update(self, x, y):
        hi_s = self.progress_s + FORWARD_SEARCH_DIST_M
        end = self.path_idx
        while end < self.n - 2 and self.dist[end + 1] <= hi_s:
            end += 1

        best = None
        for i in range(self.path_idx, end + 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            qx, qy, u, distance = self._project_segment(x, y, ax, ay, bx, by)
            segment_ds = self.dist[i + 1] - self.dist[i]
            projected_s = self.dist[i] + u * segment_ds
            if projected_s + 1e-9 < self.progress_s:
                continue
            if best is None or distance < best[0]:
                best = (distance, i, projected_s, qx, qy)

        if best is None:
            qx, qy, i = self._point_at_s(self.progress_s)
        else:
            _, i, projected_s, qx, qy = best
            self.progress_s = max(self.progress_s, projected_s)
            self.path_idx = i

        target_s = min(self.progress_s + L1, self.dist[-1])
        target_x, target_y, _ = self._point_at_s(target_s)

        tx, ty = target_x - qx, target_y - qy
        tangent_norm = math.hypot(tx, ty)
        if tangent_norm < 1e-9:
            ax, ay = self.path[self.path_idx]
            bx, by = self.path[min(self.path_idx + 1, self.n - 1)]
            tx, ty = bx - ax, by - ay
            tangent_norm = max(math.hypot(tx, ty), 1e-9)
        tx, ty = tx / tangent_norm, ty / tangent_norm

        return qx, qy, target_x, target_y, tx, ty


class CoverageGuidanceSim(Node):
    def __init__(self):
        super().__init__('coverage_guidance_sim_refined')
        self.path, self.dist = load_path(PATH_CSV, ORIGIN_X, ORIGIN_Y)
        self.n = len(self.path)
        self.guidance = PathGuidance(self.path, self.dist)
        self.pos = None
        self.z = 0.0
        self.yaw = 0.0
        self.airborne = False
        self.done = False
        self.t0 = time.time()
        self.last_target = None
        self.last_projection = None
        self.trail_x = []
        self.trail_y = []

        self.get_logger().info(
            f'경로 로드: {self.n}개 점, 총 길이 {self.dist[-1]:.2f}m, 원점({ORIGIN_X:.2f},{ORIGIN_Y:.2f})')

        self.log_file = open(LOG_CSV, 'w', newline='')
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow(['t', 'x', 'y', 'z', 'yaw_deg', 'path_idx',
                                   'target_x', 'target_y', 'eta_deg', 'cross_track_err'])

        self.create_subscription(Odometry, f'/{ROBOT_NS}/odom', self.odom_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_cb(self, msg):
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.z = msg.pose.pose.position.z
        self.yaw = quat_to_yaw(msg.pose.pose.orientation)
        if self.airborne:
            self.trail_x.append(self.pos[0])
            self.trail_y.append(self.pos[1])

    def finish(self, reason):
        self.done = True
        self.publish_vel(0.0, 0.0, 0.0)
        self.log_file.close()
        self.get_logger().info(f'{reason} -- 로그 저장됨: {LOG_CSV}')

    def control_loop(self):
        if self.pos is None or self.done:
            return

        if not self.airborne:
            if self.z < HOVER_HEIGHT:
                self.publish_vel(0.0, 0.0, 0.0, vz=0.5)
                return
            else:
                self.airborne = True

        x, y = self.pos
        gx, gy = self.path[-1]
        if (self.guidance.path_idx >= self.n - 5 and
                math.hypot(x - gx, y - gy) < GOAL_RADIUS):
            self.finish('커버리지 경로 완주')
            return

        proj_x, proj_y, ref_x, ref_y, tx, ty = self.guidance.update(x, y)
        self.last_target = (ref_x, ref_y)
        self.last_projection = (proj_x, proj_y)

        # 횡오차 P 보정 (코너 감속 없이 순항속도 V는 항상 고정)
        corr_x = CROSS_TRACK_KP * (proj_x - x)
        corr_y = CROSS_TRACK_KP * (proj_y - y)
        corr_mag = math.hypot(corr_x, corr_y)
        if corr_mag > MAX_CROSS_SPEED:
            scale = MAX_CROSS_SPEED / corr_mag
            corr_x *= scale
            corr_y *= scale

        vx_world = V * tx + corr_x
        vy_world = V * ty + corr_y

        planar_speed = math.hypot(vx_world, vy_world)
        max_planar_speed = math.hypot(V, MAX_CROSS_SPEED)
        if planar_speed > max_planar_speed:
            scale = max_planar_speed / planar_speed
            vx_world *= scale
            vy_world *= scale

        cos_yaw, sin_yaw = math.cos(self.yaw), math.sin(self.yaw)
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world

        desired_yaw = math.atan2(ty, tx)
        yaw_error = wrap_to_pi(desired_yaw - self.yaw)
        yaw_rate = clamp(YAW_KP * yaw_error,
                          -math.radians(MAX_YAW_RATE_DEG), math.radians(MAX_YAW_RATE_DEG))

        cross_track_err = math.hypot(proj_x - x, proj_y - y)
        eta = wrap_to_pi(math.atan2(ref_y - y, ref_x - x) - self.yaw)

        t = time.time() - self.t0
        self.log_writer.writerow([f'{t:.2f}', f'{x:.4f}', f'{y:.4f}',
                                   f'{self.z:.4f}', f'{math.degrees(self.yaw):.2f}',
                                   self.guidance.path_idx, f'{ref_x:.4f}', f'{ref_y:.4f}',
                                   f'{math.degrees(eta):.2f}', f'{cross_track_err:.4f}'])

        if DEBUG:
            self.get_logger().info(
                f'pos=({x:.2f},{y:.2f}) idx={self.guidance.path_idx}/{self.n} '
                f'e={cross_track_err:.3f} vx={vx_body:.2f} vy={vy_body:.2f}')

        self.publish_vel(vx_body, vy_body, yaw_rate)

    def publish_vel(self, vx, vy, wz, vz=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
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
    target_dot, = ax.plot([], [], 'ro', markersize=8, label='lookahead target')
    projection_dot, = ax.plot([], [], 'go', markersize=5, label='path projection')
    for i, (ox, oy, osize) in enumerate(OBSTACLES):
        ax.add_patch(plt.Rectangle((ox - osize / 2, oy - osize / 2), osize, osize,
                                    facecolor='red', alpha=0.4, edgecolor='darkred',
                                    label='obstacle' if i == 0 else None))
    ax.set_xlim(min(path_x) - 0.3, max(path_x) + 0.3)
    ax.set_ylim(min(path_y) - 0.3, max(path_y) + 0.3)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right')
    ax.set_title('Coverage path -- cross-track feedback (sim)')

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
            if node.last_projection is not None:
                projection_dot.set_data([node.last_projection[0]], [node.last_projection[1]])
            if node.trail_x:
                trail_line.set_data(node.trail_x, node.trail_y)

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(0.001)

            if node.done:
                if PLOT_PNG:
                    fig.savefig(PLOT_PNG, dpi=150)
                    print(f'궤적 그래프 저장됨: {PLOT_PNG}')
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
