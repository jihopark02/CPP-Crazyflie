# cf231 Coverage Guidance

OptiTrack/NatNet 모션캡쳐로 위치를 추적하는 Bitcraze Crazyflie(cf231)의 커버리지 경로 유도
시스템입니다. 실제 하드웨어 비행과 Gazebo 시뮬레이션 양쪽에서 검증했습니다.

## 구조

- **`tracking/`** — SITL과 실비행이 공유하는 플랫폼 독립적인 경로 추종 코어입니다.
  `PathGuidance`가 선분 투영, 단조 경로 진행도, lookahead 방향 및 코너 각도를 계산하고,
  `sitl/`과 `real-flight/`의 실행 파일은 각 환경의 위치 입력과 속도 명령만 담당합니다.

- **`real-flight/`** — ROS2를 거치지 않고 [cflib](https://github.com/bitcraze/crazyflie-lib-python)과
  vendored NatNet 클라이언트로 실제 드론을 직접 비행시키는 독립 실행 스크립트입니다.
  위치/자세는 Motive에서 NatNet으로 받아 `extpos`를 통해 온보드 EKF에 주입합니다.
  - `07_coverage_crosstrack_guidance_final.py` — 최종 실비행 유도 스크립트. `tracking/`의
    `PathGuidance`(선분 투영 + 횡오차 P 보정, 코너 자동감속)를 사용하고, 추정기/아밍 설정과
    전체 CSV 로깅을 포함. 모든 실비행 커버리지 테스트(obs0/obs1/obs3)에 실제로 사용된 버전
  - `plot_flight_error.py` — 비행 로그 CSV를 읽어서 cross-track/고도 오차를 시간축 그래프로 그림
  - `paths/` — 커버리지 경로 (CSV + 원본 엑셀), 장애물 회피 변형 경로 포함

- **`sitl/`** — 같은 유도 알고리즘을 Gazebo(ros_gz)에서 검증한 코드입니다.
  - `coverage_crosstrack_guidance_sim.py` — `07`과 같은 `PathGuidance`(선분투영+횡오차보정)를
    ROS2 `/cmd_vel`(Twist)로 이식한 최종 시뮬레이션 스크립트, **코너 감속 기능은 제외** —
    아래 `sitl/results/` 결과가 이 스크립트로 나온 것입니다
  - `plot_sim_error.py` — 시뮬레이션 비행 로그 오차 분석
  - `worlds/crazyflie_world.sdf` — 장애물 박스 3개가 있는 Gazebo 월드 (장애물 회피 테스트용)

- **`sitl/results/`** — 시나리오별 시뮬레이션 결과 (`obs0`: 장애물 없음, `obs1`: 장애물 1개,
  `obs3`: 장애물 3개). 각각 비행 로그(CSV + 엑셀), 계획 경로와 같은 포맷으로 재추출한
  실제 비행 궤적, 궤적 그래프(계획경로 vs 장애물 vs 실비행궤적), cross-track/고도 오차
  그래프를 담고 있습니다.

## 시뮬레이션 환경 설정

Gazebo 쪽은 이 저장소에 포함되지 않은 제3자 ROS2/Gazebo 브릿지 패키지가 필요합니다.

```bash
cd ~/ros2_ws/src
git clone https://github.com/knmcguire/ros_gz_crazyflie
cd ~/ros2_ws && colcon build --symlink-install
```

이 저장소의 `sitl/worlds/crazyflie_world.sdf`를 그 패키지의
`ros_gz_crazyflie_gazebo/share/ros_gz_crazyflie_gazebo/worlds/crazyflie_world.sdf`
(install된 복사본)에 덮어씌우면 `obs1`/`obs3` 테스트에 쓰인 넓어진 방 크기와
장애물 박스 3개가 그대로 적용됩니다.

이 패키지의 launch 파일은 Gazebo 모델 검색 경로를 설정하지 않은 채
`model://crazyflie`를 로드해서 기본 상태로는 `Unable to find uri[model://crazyflie]`
에러로 실패합니다. 실행 전에 아래처럼 리소스 경로를 지정해서 우회하세요.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/ros_gz_crazyflie/ros_gz_crazyflie_gazebo/models:$GZ_SIM_RESOURCE_PATH"
ros2 launch ros_gz_crazyflie_bringup crazyflie_simulation.launch.py
```

그 다음 다른 터미널에서:

```bash
python3 sitl/coverage_crosstrack_guidance_sim.py <path_csv> <log_csv> [plot_png] [ox,oy,size ...]
```

## 유도 알고리즘

실비행/시뮬레이션 유도 컨트롤러 모두 다음과 같이 동작합니다.
1. 드론의 현재 위치를 계획 경로의 가장 가까운 "선분"에 투영 (가장 가까운 샘플 점이 아님),
   누적거리(arc-length) 진행도 `s`를 단조증가로 추적해서 급격한 코너에서도 추종 기준점이
   뒤로 튀지 않도록 함
2. 일정한 경로방향 속도 + 비례(P) 횡오차 보정 속도(최대 횡속도로 제한)를 합성해서
   world frame에서 body frame으로 변환한 속도를 명령
3. (실비행 `07`만 해당) 전방 경로 방향이 급격히 바뀌면(코너) 자동으로 감속;
   시뮬레이션 이식 버전은 더 단순한 기준선 비교를 위해 이 기능을 뺐음

## 검증 결과 (시뮬레이션)

| 시나리오 | 장애물 | 결과 | 평균 cross-track error |
|---|---|---|---|
| obs0 | 없음 | 완주 | 약 0.8 cm |
| obs1 | 1개 | 완주, 최소 이격거리 약 22 cm | 약 1.0 cm |
| obs3 | 3개 | 완주, 최소 이격거리 약 22 cm | 약 1.1 cm |

실제 하드웨어에서도 같은 `07_coverage_crosstrack_guidance_final.py` 유도로 3m x 2m
사각형 경로와 2.5cm 해상도 커버리지 경로 여러 개를 문제없이 완주했으며, cross-track
오차는 수 센티미터 수준이었고 기울기나 위치오차로 인한 안전 컷오프는 한 번도 발생하지
않았습니다 (실비행에서 유일하게 발생한 실패 원인은 배터리 전압 저하였습니다).
