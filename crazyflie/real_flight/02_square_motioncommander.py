#!/usr/bin/env python3
"""
cf231 실비행 -- 사각형 경로, cflib 직접 + NatNetClient.py 직접 (ROS2 없음)
direct_cflib_fly.py의 검증된 이착륙/추정기 하네스 재사용, MotionCommander 바디프레임 이동으로 사각형 트레이스
기울기/저전압 감지 시 로그 콜백 스레드에서 즉시 디스암 (블로킹 이동 중에도 즉시 반응)
"""
import sys
import os
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
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7E7'
SERVER_IP = '192.168.50.49'
CLIENT_IP = '192.168.50.236'
USE_MULTICAST = False

HOVER_HEIGHT = 0.5
LEG_SPEED = 0.2   # m/s
SIDE_W = 1.0      # 가로 (m)
SIDE_H = 1.0      # 세로 (m)
MAX_TILT_DEG = 25.0
MIN_VBAT = 3.7


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

        monitor_state = {'roll': 0.0, 'pitch': 0.0, 'z': 0.0, 'vbat': 4.2}

        def pose_log_cb(timestamp, data, logconf):
            monitor_state['z'] = data['stateEstimate.z']
            monitor_state['roll'] = data['stabilizer.roll']
            monitor_state['pitch'] = data['stabilizer.pitch']
            monitor_state['vbat'] = data['pm.vbat']
            if not fault.is_set() and (
                    abs(monitor_state['roll']) > MAX_TILT_DEG or
                    abs(monitor_state['pitch']) > MAX_TILT_DEG or
                    monitor_state['vbat'] < MIN_VBAT):
                fault.set()
                print(f"비상 디스암: roll={monitor_state['roll']:.1f} "
                      f"pitch={monitor_state['pitch']:.1f} vbat={monitor_state['vbat']:.2f}")
                cf.platform.send_arming_request(False)

        pose_log = LogConfig(name='PoseMonitor', period_in_ms=50)
        pose_log.add_variable('stateEstimate.z', 'float')
        pose_log.add_variable('stabilizer.roll', 'float')
        pose_log.add_variable('stabilizer.pitch', 'float')
        pose_log.add_variable('pm.vbat', 'float')
        cf.log.add_config(pose_log)
        pose_log.data_received_cb.add_callback(pose_log_cb)
        pose_log.start()

        print('이륙...')
        with MotionCommander(scf, default_height=HOVER_HEIGHT) as mc:
            legs = [('forward', SIDE_W), ('left', SIDE_H), ('back', SIDE_W), ('right', SIDE_H)]
            for name, dist in legs:
                if fault.is_set():
                    print('비상 감지 -- 남은 구간 스킵')
                    break
                print(f'{name} {dist:.1f}m')
                getattr(mc, name)(dist, velocity=LEG_SPEED)
                print(f"  z={monitor_state['z']:.2f} roll={monitor_state['roll']:.1f} "
                      f"pitch={monitor_state['pitch']:.1f} vbat={monitor_state['vbat']:.2f}")
            # with 블록 끝나면 MotionCommander가 자동 착륙 (fault 상태면 이미 디스암돼서 사실상 무동작)

        pose_log.stop()
        stop_flag.set()
        if not fault.is_set():
            cf.platform.send_arming_request(False)

    natnet_client.shutdown()


if __name__ == '__main__':
    main()
