#!/usr/bin/env python3
"""
cf231 실비행 -- L1 Nonlinear Guidance, 커버리지 경로 (cflib 직접 + NatNetClient.py 직접, ROS2 없음)
경로는 CSV(coverage_path.csv, columns: x,y,dist)에서 로드 -- path_2.5cm_50cm.xlsx를 변환한 것
(3m x 2.5m 영역, 0.5m 서브셀, 2.5cm 점간격, 총 길이 약 13.4m).
direct_cflib_l1_square.py와 동일한 L1 유도/안전감시 하네스, 경로 로드 부분만 교체.
"""
import sys
import os
import csv
import math
import time
import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.expanduser(
    '~/natnet_ws/src/natnet_ros2/deps/NatNetSDK/samples/PythonClient'))
from NatNetClient import NatNetClient  # noqa: E402

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig

URI = 'radio://0/80/2M/E7E7E7E7E7'
SERVER_IP = '192.168.50.49'
CLIENT_IP = '192.168.50.236'
USE_MULTICAST = False

PATH_CSV = os.path.expanduser('~/natnet_ws/coverage_path.csv')
LOG_CSV = os.path.expanduser('~/natnet_ws/flight_log.csv')
V = 0.2                   # 순항 속도 (m/s)
L1 = 0.15                   # 전방주시거리 (m)
GOAL_RADIUS = 0.15         # 도착 판정 반경 (m)
FORWARD_SEARCH_DIST_M = 0.35
HOVER_HEIGHT = 0.5
CONTROL_HZ = 10.0

MAX_TILT_DEG = 25.0
MIN_VBAT = 3.7
MAX_Z = 1.2


def load_path(csv_path):
    pts, dists = [], []
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            pts.append((float(row['x']), float(row['y'])))
            dists.append(float(row['dist']))
    return pts, dists


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MocapState:
    def __init__(self):
        self.latest = None
        self.first_id = None
        self.lock = threading.Lock()

    def rigid_body_cb(self, new_id, pos, rot):
        with self.lock:
            if self.first_id is None:
                self.first_id = new_id
                print(f'[mocap] 첫 Rigid Body ID={new_id} 를 드론으로 사용')
            if new_id == self.first_id:
                self.latest = (pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3])


def start_natnet(mocap_state):
    client = NatNetClient()
    client.set_client_address(CLIENT_IP)
    client.set_server_address(SERVER_IP)
    client.set_use_multicast(USE_MULTICAST)
    client.rigid_body_listener = mocap_state.rigid_body_cb
    ok = client.run('d')
    if not ok:
        raise RuntimeError('NatNet 클라이언트 시작 실패')
    return client


def setup_estimator_params(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    time.sleep(0.1)
    cf.param.set_value('locSrv.extPosStdDev', '1e-3')
    time.sleep(0.1)
    cf.param.set_value('locSrv.extQuatStdDev', '0.5e-1')
    time.sleep(0.1)
    print('추정기 파라미터 설정 완료 (estimator=kalman, extPosStdDev=1e-3, extQuatStdDev=0.5e-1)')


def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(cf)


def wait_for_position_estimator(cf, timeout=10.0):
    log_config = LogConfig(name='KalmanVar', period_in_ms=200)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_hist = [1000] * 10
    var_y_hist = [1000] * 10
    var_z_hist = [1000] * 10
    threshold = 0.001

    t0 = time.time()
    with SyncLogger(cf, log_config) as logger:
        for entry in logger:
            data = entry[1]
            var_x_hist.append(data['kalman.varPX']); var_x_hist.pop(0)
            var_y_hist.append(data['kalman.varPY']); var_y_hist.pop(0)
            var_z_hist.append(data['kalman.varPZ']); var_z_hist.pop(0)

            if (max(var_x_hist) - min(var_x_hist) < threshold and
                    max(var_y_hist) - min(var_y_hist) < threshold and
                    max(var_z_hist) - min(var_z_hist) < threshold):
                print('추정기 수렴 완료')
                return True
            if time.time() - t0 > timeout:
                print('수렴 타임아웃 -- 그래도 진행')
                return False


class L1Guidance:
    """coverage_guidance_sim_live.py와 동일: path_idx는 절대 뒤로 안 감(단조증가).
    U턴이 많은 커버리지 경로에서 path_idx가 반대편(이미 지나온 줄)으로 튀는 것 방지."""

    def __init__(self, path, dist):
        self.path = path
        self.dist = dist
        self.n = len(path)
        self.path_idx = 0
        self.nearest_dist = 0.0

    def update_nearest_idx(self, x, y):
        cur_dist = self.dist[self.path_idx]
        hi = cur_dist + FORWARD_SEARCH_DIST_M
        end = self.path_idx
        while end < self.n - 1 and self.dist[end + 1] <= hi:
            end += 1
        best_dist, best_i = float('inf'), self.path_idx
        for i in range(self.path_idx, end + 1):
            px, py = self.path[i]
            d = math.hypot(px - x, py - y)
            if d < best_dist:
                best_dist, best_i = d, i
        self.path_idx = best_i
        self.nearest_dist = best_dist  # cross-track error 근사값 (경로점 간격 2.5cm 이내 정밀도)

    def find_l1_point(self, x, y):
        self.update_nearest_idx(x, y)
        for i in range(self.path_idx, self.n):
            px, py = self.path[i]
            if math.hypot(px - x, py - y) >= L1:
                return px, py
        return self.path[-1]


def main():
    mocap = MocapState()
    print('NatNet 연결 시도...')
    natnet_client = start_natnet(mocap)

    print('mocap 데이터 대기 중...')
    t_wait0 = time.time()
    while mocap.latest is None:
        time.sleep(0.05)
        if time.time() - t_wait0 > 10.0:
            print('mocap 데이터가 10초 넘게 안 들어옴 -- Motive에서 cf231 Solved 상태인지 확인하세요')
            return
    print('mocap 데이터 수신 시작:', mocap.latest)

    path, dist = load_path(PATH_CSV)
    print(f'경로 로드: {len(path)}개 점, 총 길이 {dist[-1]:.2f}m')
    guidance = L1Guidance(path, dist)

    log_file = open(LOG_CSV, 'w', newline='')
    log_writer = csv.writer(log_file)
    log_writer.writerow(['t', 'x', 'y', 'z', 'yaw_deg', 'path_idx',
                          'target_x', 'target_y', 'eta_deg', 'cross_track_err'])

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 7))
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    ax.plot(path_x, path_y, 'b-', linewidth=1.0, alpha=0.5, label='planned path')
    trail_line, = ax.plot([], [], '-', color='gray', linewidth=1.5, label='flown trail')
    drone_dot, = ax.plot([], [], 'ko', markersize=10, label='drone')
    target_dot, = ax.plot([], [], 'ro', markersize=8, label='L1 target')
    margin = 0.3
    ax.set_xlim(min(path_x) - margin, max(path_x) + margin)
    ax.set_ylim(min(path_y) - margin, max(path_y) + margin)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.legend(loc='upper right')
    ax.set_title('cf231 coverage path — live view')
    trail_x, trail_y = [], []

    def update_plot():
        trail_x.append(state['x'])
        trail_y.append(state['y'])
        drone_dot.set_data([state['x']], [state['y']])
        trail_line.set_data(trail_x, trail_y)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        setup_estimator_params(cf)

        print('아밍...')
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        stop_flag = threading.Event()
        fault = threading.Event()

        def extpose_loop():
            while not stop_flag.is_set():
                if mocap.latest is not None:
                    x, y, z, qx, qy, qz, qw = mocap.latest
                    cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
                time.sleep(0.02)

        extpose_thread = threading.Thread(target=extpose_loop, daemon=True)
        extpose_thread.start()

        time.sleep(1.0)
        print('추정기 리셋 + 수렴 대기...')
        reset_estimator(cf)

        state = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0, 'roll': 0.0, 'pitch': 0.0, 'vbat': 4.2}

        def pose_log_cb(timestamp, data, logconf):
            state['x'] = data['stateEstimate.x']
            state['y'] = data['stateEstimate.y']
            state['z'] = data['stateEstimate.z']
            state['yaw'] = data['stabilizer.yaw']

        def safety_log_cb(timestamp, data, logconf):
            state['roll'] = data['stabilizer.roll']
            state['pitch'] = data['stabilizer.pitch']
            state['vbat'] = data['pm.vbat']
            if not fault.is_set() and (
                    abs(state['roll']) > MAX_TILT_DEG or
                    abs(state['pitch']) > MAX_TILT_DEG or
                    state['vbat'] < MIN_VBAT or
                    state['z'] > MAX_Z):
                fault.set()
                print(f"비상 감지: roll={state['roll']:.1f} pitch={state['pitch']:.1f} "
                      f"vbat={state['vbat']:.2f} z={state['z']:.2f}")

        pose_log = LogConfig(name='PoseMonitor', period_in_ms=50)
        pose_log.add_variable('stateEstimate.x', 'float')
        pose_log.add_variable('stateEstimate.y', 'float')
        pose_log.add_variable('stateEstimate.z', 'float')
        pose_log.add_variable('stabilizer.yaw', 'float')
        cf.log.add_config(pose_log)
        pose_log.data_received_cb.add_callback(pose_log_cb)
        pose_log.start()

        safety_log = LogConfig(name='SafetyMonitor', period_in_ms=50)
        safety_log.add_variable('stabilizer.roll', 'float')
        safety_log.add_variable('stabilizer.pitch', 'float')
        safety_log.add_variable('pm.vbat', 'float')
        cf.log.add_config(safety_log)
        safety_log.data_received_cb.add_callback(safety_log_cb)
        safety_log.start()

        def emergency_stop(reason):
            print(f'비상 정지: {reason}')
            cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            cf.platform.send_arming_request(False)

        dt = 1.0 / CONTROL_HZ

        try:
            # ---- 이륙 ----
            print('이륙...')
            t_takeoff0 = time.time()
            target_z = 0.0
            while True:
                if fault.is_set():
                    emergency_stop('이륙 중 이상 감지')
                    return
                if time.time() - t_takeoff0 > 8.0:
                    emergency_stop('이륙 타임아웃')
                    return
                target_z = min(HOVER_HEIGHT, target_z + 0.3 * dt)
                cf.commander.send_hover_setpoint(0, 0, 0, target_z)
                if target_z >= HOVER_HEIGHT and state['z'] > HOVER_HEIGHT - 0.05:
                    break
                update_plot()
                plt.pause(dt)

            # ---- L1 유도 커버리지 경로 추종 ----
            print('L1 유도 시작')
            t_mission0 = time.time()
            gx, gy = path[-1]
            while True:
                if fault.is_set():
                    emergency_stop('유도 중 이상 감지')
                    return
                if time.time() - t_mission0 > 180.0:
                    emergency_stop('미션 타임아웃')
                    return

                x, y, yaw_deg = state['x'], state['y'], state['yaw']
                if (guidance.path_idx >= guidance.n - 5 and
                        math.hypot(x - gx, y - gy) < GOAL_RADIUS):
                    print('경로 완주')
                    break

                ref_x, ref_y = guidance.find_l1_point(x, y)
                bearing = math.atan2(ref_y - y, ref_x - x)
                eta = wrap_to_pi(bearing - math.radians(yaw_deg))
                a_s = 2 * V ** 2 / L1 * math.sin(eta)
                psi_dot = math.degrees(a_s / V)

                cf.commander.send_hover_setpoint(V, 0, psi_dot, HOVER_HEIGHT)
                print(f'  idx={guidance.path_idx}/{guidance.n} x={x:.2f} y={y:.2f} '
                      f'z={state["z"]:.2f} eta={math.degrees(eta):.1f}')
                log_writer.writerow([time.time() - t_mission0, x, y, state['z'], yaw_deg,
                                      guidance.path_idx, ref_x, ref_y,
                                      math.degrees(eta), guidance.nearest_dist])
                log_file.flush()
                target_dot.set_data([ref_x], [ref_y])
                update_plot()
                plt.pause(dt)

            # ---- 착륙 ----
            print('착륙...')
            t_land0 = time.time()
            target_z = HOVER_HEIGHT
            while target_z > 0.02:
                if fault.is_set():
                    emergency_stop('착륙 중 이상 감지')
                    return
                if time.time() - t_land0 > 10.0:
                    emergency_stop('착륙 타임아웃')
                    return
                target_z = max(0.0, target_z - 0.2 * dt)
                cf.commander.send_hover_setpoint(0, 0, 0, target_z)
                update_plot()
                plt.pause(dt)

            cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            cf.platform.send_arming_request(False)
            print('완료 -- 디스암')

        finally:
            stop_flag.set()
            log_file.close()
            print(f'비행 로그 저장됨: {LOG_CSV}')

    natnet_client.shutdown()

    plt.ioff()
    print('비행 종료 -- 그래프 창을 닫으면 프로그램이 끝납니다')
    plt.show()


if __name__ == '__main__':
    main()
