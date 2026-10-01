from grid_map import GridMap

# 작업 공간 크기 (미터). 가로 3.0m x 세로 2.5m.
MAP_WIDTH_M = 3.0
MAP_HEIGHT_M = 2.5

# 에이전트(로봇) 크기 = 커버리지 툴 크기 D (미터).
AGENT_SIZE_M = 0.5

# 그리드 한 칸(subcell) = 에이전트 크기와 동일한 D x D 정사각형.
# 3.0 / 0.5 = 6 cols, 2.5 / 0.5 = 5 rows.
ROWS = round(MAP_HEIGHT_M / AGENT_SIZE_M)
COLS = round(MAP_WIDTH_M / AGENT_SIZE_M)

START_POS = (ROWS - 1, 0)  # 로봇/커버리지 경로 시작 위치: 왼쪽 아래 (x=0, y=0)

# 화면에서 subcell 하나를 몇 픽셀로 그릴지 (창 크기 조절용). 키우면 창이 커진다.
# 창 크기 대략: 가로 = max(COLS*SUBCELL_PX + 100, MIN_WINDOW_WIDTH_PX), 세로 = ROWS*SUBCELL_PX + 100 + PANEL_HEIGHT_PX
SUBCELL_PX = 100

# 창의 최소 가로 크기(픽셀). 맵이 작아서 그리드 폭이 이보다 좁아도 창은 이
# 너비 이상으로 열려서, 하단 버튼 줄과 상태 메시지가 잘리지 않는다.
MIN_WINDOW_WIDTH_PX = 1000

# 하단 패널 높이(픽셀). 버튼 한 줄 + 상태 메시지 약 3줄이 들어가는 높이.
PANEL_HEIGHT_PX = 120

# 에이전트의 최소 회전반경 r_min (미터). agriculture 논문(Höffmann et al.,
# 2023)의 곡률 제약(kappa_max = 1/r_min)에 해당 -- path_smoothing.py가 이
# 반경 이하의 원호로 경로의 모든 모서리를 둥글린다. 값을 낮추면(에이전트가
# 더 잘 돌면) 경로가 원래 grid 경로에 더 가까워지고, 높이면 더 넓게 둥글린
# 경로가 된다. subcell 한 칸(AGENT_SIZE_M)보다 훨씬 크면 좁은 셀 안에서는
# 반경을 다 못 채우고 잘리는(tight-radius) 모서리가 늘어난다.
MIN_TURN_RADIUS_M = 0.3

# 모서리 하나를 원호로 그릴 때 몇 개의 점으로 근사할지 (많을수록 매끄럽게
# 보이지만 계산량이 늘어난다).
ARC_SAMPLES_PER_CORNER = 12

# Save Path 버튼으로 경로를 저장할 때, 스무딩된 경로를 따라 점을 몇 미터
# 간격으로 찍어서 저장할지 (미터). 0.025 = 2.5cm.
PATH_POINT_SPACING_M = 0.025

# Coverage Path 애니메이션 속도 (초당 이동 거리, 미터). 값을 낮추면 에이전트가
# 더 천천히 움직인다. 즉시 전체 경로를 보고 싶으면 아주 크게(예: 999) 설정.
ANIM_SPEED_M_PER_SEC = 10.0

# TSP-CPP Genetic Algorithm 파라미터 (tsp_cpp_ga.solve_tsp_cpp에 그대로
# 전달됨). Coverage Path 버튼을 누를 때마다 셀 방문 순서와 각 셀의 진입/이탈
# 조합(스윕 방향 2 x 코너 4 = 8가지 중 하나)을 이 설정 하나의 GA로 동시에 푼다 -- Tung & Liu (2019)
# 논문 구조 + WJTung/GA-TSPCPP(GA.cpp) 구현 참고: (cell*4+choice) 결합
# 유전자, 룰렛휠 선택, heuristic crossover, swap mutation, 그리고 각 개체마다
# 고정된 방문 순서에 대해 최적 진입/이탈 조합을 DP로 찾는 서브루틴(논문
# Algorithm 1). 셀 진출점과 다음 셀 진입점이 인접하지 않으면 전이 비용을
# A* 경로 길이로 계산한다.
GA_PARAMS = dict(
    population_size=100,       # 세대(generation)당 개체 수
    generations=200,           # 최대 세대 수
    crossover_rate=0.9,        # 자식을 heuristic crossover로 만들 확률 (아니면 복제)
    swap_mutation_rate=0.1,    # 개체별 "방문 순서 두 유전자 스왑" 확률
    elite_size=4,              # 다음 세대로 그대로 넘기는 상위 개체 수
    patience=50,               # 이만큼 세대 동안 개선이 없으면 조기 종료
    # 셀 내부 스윕 방향(세로/가로 x 4코너 = 8가지 조합)도 같이 최적화한다.
    # turn_penalty는 셀 내부 회전 1번당 추가 비용(그리드 칸 단위): 작으면
    # 길이가 같은 조합 사이의 tie-breaker, 크게(예: 0.5) 하면 경로가 조금
    # 길어지더라도 회전이 적은 스윕 방향을 선호한다.
    turn_penalty=0.01,
    # GA는 무작위 초기 population에서 출발해서 탐색하기 때문에, seed를 고정하지
    # 않으면(None) 장애물 배치가 똑같아도 Coverage Path를 다시 계산할 때마다
    # 조금씩 다른 결과가 나올 수 있다. 매번 같은 경로를 재현하고 싶으면 정수로
    # 고정(예: 42)하고, 매번 다른 해를 탐색해 더 나은 결과를 찾고 싶으면
    # None으로 둔다.
    seed=42,
)


def main():
    grid_map = GridMap(
        rows=ROWS, cols=COLS,
        subcell_size_m=AGENT_SIZE_M,
        subcell_px=SUBCELL_PX,
        min_window_width=MIN_WINDOW_WIDTH_PX,
        panel_height=PANEL_HEIGHT_PX,
        start_pos=START_POS,
        anim_speed_m_per_sec=ANIM_SPEED_M_PER_SEC,
        ga_params=GA_PARAMS,
        min_turn_radius_m=MIN_TURN_RADIUS_M,
        arc_samples_per_corner=ARC_SAMPLES_PER_CORNER,
        path_point_spacing_m=PATH_POINT_SPACING_M,
    )
    grid_map.run()


if __name__ == "__main__":
    main()
