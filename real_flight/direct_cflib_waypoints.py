#!/usr/bin/env python3
"""
cf231 실비행 -- 웨이포인트 순회, cflib 직접 + NatNetClient.py 직접 (ROS2 없음)
위치(모캡) -> PID -> world-frame 속도 명령. 항상 이동 방향을 바라보도록 요우 제어.
각 웨이포인트 반경 0.1m 안에 들어오면 도착 판정 -> 5초 호버 -> 다음 웨이포인트.
"""
import sys
import os
import math
import time
import threading

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

HOVER_HEIGHT = 0.5
WAYPOINTS = [(3.0, 0.0), (3.0, 2.0), (2.0, 0.0), (0.0, 0.0)]
ARRIVE_RADIUS = 0.1
HOVER_TIME = 5.0
MAX_SPEED = 0.5       # m/s, 수평 속도 saturation
MAX_VZ = 0.3           # m/s, 수직 속도 saturation
MAX_YAWRATE_DEG = 120.0
CONTROL_HZ = 20.0

MAX_TILT_DEG = 25.0
MIN_VBAT = 3.7

# PID gains: 위치오차(m) -> 속도(m/s)
KP_XY, KI_XY, KD_XY = 0.8, 0.02, 0.3
KP_Z = 0.8
YAW_KP = 2.0  # rad/s per rad 오차


def wrap_to_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PID:
    def __init__(self, kp, ki, kd, i_limit=1.0):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def update(self, error, now):
        dt = (1.0 / CONTROL_HZ) if self.prev_time is None else max(1e-3, now - self.prev_time)
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + error * dt))
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        self.prev_time = now
        return self.kp * error + self.ki * self.integral + self.kd * derivative


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
                    state['vbat'] < MIN_VBAT):
                fault.set()
                print(f"비상 감지: roll={state['roll']:.1f} pitch={state['pitch']:.1f} "
                      f"vbat={state['vbat']:.2f}")

        # 로그 블록 하나에 변수를 너무 많이 넣으면 cflib가
        # "log configuration is too large"로 거부해서 두 블록으로 나눔
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
            # ---- 이륙 (수직 상승) ----
            print('이륙...')
            t_takeoff0 = time.time()
            while state['z'] < HOVER_HEIGHT - 0.05:
                if fault.is_set():
                    emergency_stop('이륙 중 이상 감지')
                    return
                if time.time() - t_takeoff0 > 5.0:
                    emergency_stop('이륙 타임아웃 -- 목표 고도 도달 못함')
                    return
                cf.commander.send_velocity_world_setpoint(0, 0, 0.3, 0)
                time.sleep(dt)

            # ---- 웨이포인트 순회 ----
            hold_yaw_rad = math.radians(state['yaw'])
            for wp_x, wp_y in WAYPOINTS:
                print(f'-> 웨이포인트 ({wp_x:.1f}, {wp_y:.1f})')
                x_pid, y_pid, z_pid = PID(KP_XY, KI_XY, KD_XY), PID(KP_XY, KI_XY, KD_XY), PID(KP_Z, 0.0, 0.2)

                # 도착할 때까지 이동 (경로 길이 감안 넉넉한 타임아웃)
                t_leg0 = time.time()
                while True:
                    if fault.is_set():
                        emergency_stop('이동 중 이상 감지')
                        return
                    if time.time() - t_leg0 > 30.0:
                        emergency_stop(f'웨이포인트 ({wp_x:.1f},{wp_y:.1f}) 도착 타임아웃')
                        return
                    now = time.time()
                    dx, dy = wp_x - state['x'], wp_y - state['y']
                    dist = math.hypot(dx, dy)
                    if dist < ARRIVE_RADIUS:
                        break

                    vx = x_pid.update(dx, now)
                    vy = y_pid.update(dy, now)
                    speed = math.hypot(vx, vy)
                    if speed > MAX_SPEED:
                        vx, vy = vx * MAX_SPEED / speed, vy * MAX_SPEED / speed
                    vz = max(-MAX_VZ, min(MAX_VZ, z_pid.update(HOVER_HEIGHT - state['z'], now)))

                    hold_yaw_rad = math.atan2(dy, dx)
                    yaw_err = wrap_to_pi(hold_yaw_rad - math.radians(state['yaw']))
                    yawrate = max(-MAX_YAWRATE_DEG, min(MAX_YAWRATE_DEG, math.degrees(YAW_KP * yaw_err)))

                    cf.commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)
                    print(f"  x={state['x']:.2f} y={state['y']:.2f} z={state['z']:.2f} "
                          f"dist={dist:.2f} vx={vx:.2f} vy={vy:.2f}")
                    time.sleep(dt)

                print(f'도착 -- {HOVER_TIME:.0f}초 호버')
                hover_pid_x, hover_pid_y, hover_pid_z = PID(KP_XY, KI_XY, KD_XY), PID(KP_XY, KI_XY, KD_XY), PID(KP_Z, 0.0, 0.2)
                t_hover0 = time.time()
                while time.time() - t_hover0 < HOVER_TIME:
                    if fault.is_set():
                        emergency_stop('호버 중 이상 감지')
                        return
                    now = time.time()
                    vx = hover_pid_x.update(wp_x - state['x'], now)
                    vy = hover_pid_y.update(wp_y - state['y'], now)
                    speed = math.hypot(vx, vy)
                    if speed > MAX_SPEED:
                        vx, vy = vx * MAX_SPEED / speed, vy * MAX_SPEED / speed
                    vz = max(-MAX_VZ, min(MAX_VZ, hover_pid_z.update(HOVER_HEIGHT - state['z'], now)))
                    yaw_err = wrap_to_pi(hold_yaw_rad - math.radians(state['yaw']))
                    yawrate = max(-MAX_YAWRATE_DEG, min(MAX_YAWRATE_DEG, math.degrees(YAW_KP * yaw_err)))
                    cf.commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)
                    time.sleep(dt)

            # ---- 착륙 (수직 하강) ----
            print('착륙...')
            t_land0 = time.time()
            while state['z'] > 0.05:
                if fault.is_set():
                    emergency_stop('착륙 중 이상 감지')
                    return
                if time.time() - t_land0 > 10.0:
                    emergency_stop('착륙 타임아웃')
                    return
                cf.commander.send_velocity_world_setpoint(0, 0, -0.2, 0)
                time.sleep(dt)

            cf.commander.send_velocity_world_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            cf.platform.send_arming_request(False)
            print('완료 -- 디스암')

        finally:
            stop_flag.set()

    natnet_client.shutdown()


if __name__ == '__main__':
    main()
