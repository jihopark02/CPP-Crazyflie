# CPP-Crazyflie

Coverage Path Planning(CPP)으로 생성한 경로를 Crazyflie에서 시뮬레이션(SITL)과 실비행으로 검증하는 통합 프로젝트입니다.

## 프로젝트 구성

```text
.
├── cpp/                     # GA-TSP 기반 Coverage Path Planning
└── crazyflie/
    ├── sitl/                # Gazebo/ROS 2 시뮬레이션 및 결과
    │   └── results/
    └── real-flight/         # Crazyflie 실비행 코드와 경로
```

## 모듈

### `cpp/`

사다리꼴 셀 분해, GA 기반 TSP-CPP, A* 연결, 곡률 제약 스무딩으로 커버리지 경로를 생성합니다. 자세한 사용법은 [`cpp/README.md`](cpp/README.md)를 참고하세요.

### `crazyflie/sitl/`

CPP에서 생성한 경로를 ROS 2와 Gazebo 환경에서 추종하고 검증합니다. 시나리오별 비행 로그와 그래프는 `crazyflie/sitl/results/`에 있습니다.

### `crazyflie/real-flight/`

cflib과 OptiTrack/NatNet을 사용해 실제 Crazyflie에서 경로 추종을 검증합니다. 단계별 비행 스크립트와 시험 경로를 포함합니다.

Crazyflie 모듈의 설정과 실행법은 [`crazyflie/README.md`](crazyflie/README.md)를 참고하세요.

## 원본 저장소

이 저장소는 다음 두 프로젝트를 하나의 구조로 통합하며, 각 원본 저장소의 Git 커밋 이력을 보존합니다.

- CPP: [jwmc1118/Coverage-Path-Planning-with-GA-TSP](https://github.com/jwmc1118/Coverage-Path-Planning-with-GA-TSP)
- Crazyflie 검증: [nyeon8973-prog/cf231-coverage-guidance](https://github.com/nyeon8973-prog/cf231-coverage-guidance)

