"""통합 TSP + 커버리지 경로 계획(TSP-CPP) 문제를 위한 유전 알고리즘.

Tung & Liu, "Solution of an Integrated Traveling Salesman and Coverage
Path Planning Problem by Using a Genetic Algorithm with Modified
Operators" (IADIS Int. J. Computer Science and Information Systems,
14(2), 2019)와 참조 구현 https://github.com/WJTung/GA-TSPCPP (GA.cpp)를
Python으로 다시 구현한 것이다.

  * 셀 하나당 결합 유전자 하나. 염색체는 길이 n 리스트이고 i번째
    유전자는 ``cell_id * NUM_COMBOS + choice`` (``choice``는 셀 내부
    boustrophedon 스윕이 어느 모서리로 들어오고 나가는지를 나타내는
    "cell-path combination", 논문 Figure 2). 리스트는 n개 셀의 순열이며,
    리스트 안의 위치가 곧 방문 순서다.
  * 룰렛휠(비용의 역수) 선택.
  * Heuristic crossover (참조 코드 ``heuristic_crossover``): 자식을 셀
    하나씩 늘려 간다. 매 단계 후보 셀은 *각* 부모의 방문 순서에서 현재
    셀의 왼쪽/오른쪽으로 가장 가까운, 아직 방문하지 않은 셀들(최대 4개)이고,
    각 후보를 모든 조합으로 시도해서 (현재 이탈점 -> 후보 진입점) 비용이
    가장 작은 것을 고른다.
  * Swap mutation (참조 코드 ``swap_mutation``): 가끔 두 유전자를 서로
    바꾼다.
  * 최적 진입/이탈 조합 (참조 코드 ``best_choice_combination``, 논문
    Algorithm 1). 개체의 방문 순서가 *고정되어 있을 때*, 동적 계획법으로
    전체 투어를 최소화하는 각 셀의 조합을 찾는다. 매 세대 모든 개체에
    적용하므로 GA는 사실상 셀 순서만 탐색하면 되고, 조합은 그 순서에 대해
    항상 최적이다.

참조 코드와의 의도적인 차이:

  1. 고정된 ``start_pos``(로봇의 시작 칸)에서 출발하는 열린 투어이며,
     원점으로 돌아올 필요가 없다 -- 참조 코드는 닫힌 TSP 순환을 푼다.
  2. 한 셀의 이탈점과 다음 셀의 진입점 사이의 전이 비용은 장애물을
     고려한다: 두 점이 상하좌우로 바로 붙어 있지 않으면 그리드
     A*(astar.find_path)로 연결하고, 장애물을 뚫고 지나갈 수 있는 직선
     점프 대신 그 경로의 길이를 비용으로 쓴다. 이는 참조 코드의
     visibility-graph 최단 경로를 그리드 맵에 맞게 옮긴 것이며, GA가 최종
     이어붙인 경로와 같은 거리를 최적화하게 만든다.
  3. 스윕 방향도 함께 최적화한다. 논문/참조 코드는 스윕 방향이 고정된
     4가지 조합만 쓰지만, 여기서는 세로 스윕(열 단위 왕복, 왼쪽<->오른쪽
     진행) 4가지 + 가로 스윕(행 단위 왕복, 아래<->위 진행) 4가지 = 8가지
     조합을 둔다 (NUM_COMBOS = 8). 방향에 따라 진입/이탈점이 달라지므로
     같은 DP / GA가 방향까지 같이 고른다.

같은 방향의 4가지 조합은 길이가 같지만(같은 경로를 뒤집거나 거울상으로 만든
것), 방향이 다르면 셀 모양에 따라 내부 길이와 회전 수가 달라진다. 그래서
조합별 내부 비용(길이 + turn_penalty * 회전 수)을 투어 비용에 포함한다.
직사각형 셀에서는 두 방향의 길이가 같으므로 turn_penalty가 회전이 적은
(긴 직선이 많은) 방향을 고르게 하는 tie-breaker 역할을 한다.
"""

import math
import random
from dataclasses import dataclass, field

import astar

COMBOS_PER_AXIS = 4
NUM_COMBOS = 2 * COMBOS_PER_AXIS  # 0..3: 세로 스윕, 4..7: 가로 스윕


def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def boustrophedon_variant(cell, combo):
    """cell-path combination을 실제로 구현한 부분.

    combo 0..3: 세로 스윕 (열 단위로 위아래 왕복, 열을 왼쪽/오른쪽으로 진행)
    combo 4..7: 가로 스윕 (행 단위로 좌우 왕복, 행을 아래/위로 진행)
    """
    horizontal, k = divmod(combo, COMBOS_PER_AXIS)
    reverse_lines = k in (2, 3) # 세로: 오른쪽 -> 왼쪽 / 가로: 위 -> 아래 로 진행
    start_high = k in (0, 2) # 세로: 각 열을 위에서 시작 / 가로: 각 행을 오른쪽에서 시작

    # cell.gridpoints는 그 cell에 속한 모든 격자점들의 좌표 목록을 나타냄.
    # 스윕 라인(세로면 열, 가로면 행) 별로 격자점들을 모음
    by_line = {}
    for r, c in cell.gridpoints:
        if horizontal:
            by_line.setdefault(r, []).append(c)
        else:
            by_line.setdefault(c, []).append(r)
    # 기본 진행 방향: 세로는 왼쪽 -> 오른쪽(col 증가), 가로는 아래 -> 위(row 감소)
    lines = sorted(by_line, reverse=bool(horizontal))
    if reverse_lines:
        lines.reverse()

    # 지그재그 경로 만들기
    path = []
    high_first = start_high
    for line in lines:
        # 세로: "high" = 위쪽 = 작은 row / 가로: "high" = 오른쪽 = 큰 col
        vals = sorted(by_line[line], reverse=bool(horizontal))
        if not high_first:
            vals.reverse()
        path.extend((v, line) if not horizontal else (line, v) for v in vals)
        high_first = not high_first # 매 라인마다 방향이 자동으로 뒤집힘
    return path


def _count_turns(path):
    """경로에서 진행 방향이 바뀌는 횟수 (스무딩 시 모서리 수에 해당)."""
    turns = 0
    prev = None
    for i in range(len(path) - 1):
        d = (path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        if prev is not None and d != prev:
            turns += 1
        prev = d
    return turns


@dataclass
class CellPaths:
    """하나의 cell에 대해 8가지 cell-path combination의 경로, 진입점, 이탈점, 비용을 전부 미리 계산해서 저장해두는 캐시 박스"""
    cell_id: int
    paths: list = field(default_factory=list)   # 길이 8: gridpoint 리스트들
    entries: list = field(default_factory=list)  # 길이 8: 진입점 (row, col)
    exits: list = field(default_factory=list)    # 길이 8: 이탈점 (row, col)
    lengths: list = field(default_factory=list)  # 길이 8: 셀 내부 경로 길이
    turns: list = field(default_factory=list)    # 길이 8: 셀 내부 회전 수
    costs: list = field(default_factory=list)    # 길이 8: lengths + turn_penalty * turns

    @staticmethod
    def build(cell, dist=euclidean, turn_penalty=0.0): # 실제로 값을 채우는 로직
        paths = [boustrophedon_variant(cell, k) for k in range(NUM_COMBOS)] # 이 cell의 8가지 조합 경로를 전부 만듦
        entries = [p[0] for p in paths]
        exits = [p[-1] for p in paths]
        # 라인 사이 이동이 인접하지 않으면(셀 모양이 들쭉날쭉한 경우) dist가 A*
        # 길이를 돌려주므로, 이어붙이기 단계에서 실제로 생기는 우회까지 반영된다.
        lengths = [sum(dist(p[i], p[i + 1]) for i in range(len(p) - 1)) for p in paths]
        turns = [_count_turns(p) for p in paths]
        costs = [l + turn_penalty * t for l, t in zip(lengths, turns)]
        return CellPaths(cell.id, paths, entries, exits, lengths, turns, costs)


# ---------------------------------------------------------------------------
# 두 그리드 점 사이의 장애물 인식 전이 비용
# ---------------------------------------------------------------------------

def _make_distance_fn(is_free, rows, cols):
    """두 점 사이 거리를 구하는 함수를 만들어서 반환하는 팩토리 함수임
       논문에서 말한 visibility graph 역할을 하는 부분인데, 여기서는 그리드 맵에서 구현하므로 visibility graph 대신 A*알고리즘을 사용함
    """

    cache = {} # 같은 (a, b) 쌍의 거리를 한 번 계산했으면 다시 계산 안 하고 저장된 값을 재사용, GA는 수만 번 거리 계산을 반복하니까, 같은 entry-exit 쌍이 여러 chromosome에서 반복 등장할 가능성이 큼

    def dist(a, b):

        # 같은 점일 경우, 거리는 0
        if a == b:
            return 0.0
        key = (a, b)
        hit = cache.get(key)
        if hit is not None:
            return hit[1]

        # 맵 정보가 없거나, 이미 바로 붙어있는 경우 -> 직선거리
        manhattan = abs(a[0] - b[0]) + abs(a[1] - b[1])
        if is_free is None or manhattan <= 1:
            path = [a, b]
            length = math.hypot(a[0] - b[0], a[1] - b[1])


        else: # 떨어져 있고, 맵 정보가 있는 경우 -> A* 실행
            grid_path = astar.find_path(is_free, rows, cols, a, b)
            if grid_path is None:
                path = [a, b]
                length = math.hypot(a[0] - b[0], a[1] - b[1])
            else:
                path = grid_path
                length = float(len(grid_path) - 1)  # 상하좌우 한 칸 이동 횟수

        cache[key] = (path, length)
        return length

    return dist, cache


# ---------------------------------------------------------------------------
# 비용 계산
# ---------------------------------------------------------------------------

def _genes_tour_length(genes, cell_paths, start_pos, dist):
    """GA를 쓸 때 cell + 조합을 하나로 합친 gene 방식"""
    c0, k0 = divmod(genes[0], NUM_COMBOS)
    total = dist(start_pos, cell_paths[c0].entries[k0]) + cell_paths[c0].costs[k0]
    for i in range(len(genes) - 1):
        ca, ka = divmod(genes[i], NUM_COMBOS)
        cb, kb = divmod(genes[i + 1], NUM_COMBOS)
        total += dist(cell_paths[ca].exits[ka], cell_paths[cb].entries[kb]) + cell_paths[cb].costs[kb] # 전이 비용 + 조합별 셀 내부 비용
    return total


# ---------------------------------------------------------------------------
# GA 연산자
# ---------------------------------------------------------------------------

def _random_genes(n, rng): # 완전 무작위 chromosome 만들기
    genes = [c * NUM_COMBOS + rng.randrange(NUM_COMBOS) for c in range(n)] # cell 0부터 n-1까지 각각 cell + 무작위 조합을 인코딩
    rng.shuffle(genes) # 리스트 자리를 완전히 무작위로 섞음
    return genes


def _roulette_prefix(costs): # 룰렛휠 확률표 만들기
    """룰렛휠 선택용으로, 비용 역수의 누적합을 정규화한 값 (비용이
    낮을수록 룰렛에서 차지하는 칸이 크다)."""
    prefix = []
    running = 0.0
    for c in costs:
        running += 1.0 / c if c > 0 else 1e12 # 비용의 역수를 취해서 작을수록 값이 커지게 함 -> 좋은 해일수록 룰렛판에서 넓은 칸을 차지
        prefix.append(running)
    total = prefix[-1]
    return [p / total for p in prefix]


def _roulette_pick(prefix, rng): # 화살을 던져서 하나 뽑기
    r = rng.random()
    lo, hi = 0, len(prefix) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if prefix[mid] < r:
            lo = mid + 1
        else:
            hi = mid
    return lo


def _heuristic_crossover(parent_a, parent_b, cell_paths, dist, rng):
    """참조 코드 ``heuristic_crossover``를 결합 유전자 인코딩에 맞게 옮긴
    것: 자식을 셀 하나씩 늘려 간다. 다음 셀의 후보는 *각* 부모의 방문
    순서에서 현재 셀의 양쪽(왼쪽 / 오른쪽)으로 가장 가까운, 아직 방문하지
    않은 셀들(최대 4개)이다. 각 후보를 모든 조합으로 시도해서 진입점이
    현재 이탈점에 가장 가까운 것을 고른다."""

    # 위치 인덱스 테이블 만들기
    # 염색체 내 각 cell의 방문 순서를 빠르게 구하기 위함
    n = len(parent_a)
    pos_a = [0] * n
    pos_b = [0] * n
    for idx, g in enumerate(parent_a):
        pos_a[g // NUM_COMBOS] = idx
    for idx, g in enumerate(parent_b):
        pos_b[g // NUM_COMBOS] = idx

    # 시작 cell 정하고 조합째로 복사
    visited = [False] * n
    start_cell = rng.randrange(n)
    first = parent_a[pos_a[start_cell]] if rng.random() < 0.5 else parent_b[pos_b[start_cell]] # 부모 1 또는 부모 2 중 하나에서 그 cell이 어떤 조합이었는지 통째로 가져옴
    child = [first] # 자식의 첫 유전자로
    visited[start_cell] = True

    # left-a, right-a, left-b, right-b -> 4개의 현재 탐색 위치 포인터
    la = ra = pos_a[start_cell]
    lb = rb = pos_b[start_cell]

    for _ in range(1, n): # 2, 3단계를 n-1번 반복

        # 방금 자식에 넣은 마지막 유전자를 디코딩해서 그 cell의 exit point를 구함
        pc, pk = divmod(child[-1], NUM_COMBOS)
        prev_exit = cell_paths[pc].exits[pk]


        while visited[parent_a[la] // NUM_COMBOS]: # 부모a를 대상으로 첫 유전자의 왼쪽으로 한 칸씩 이동하면서 이미 방문한 cell이면 계속 건넘뜀
            la = (la - 1) % n
        while visited[parent_a[ra] // NUM_COMBOS]: # '' 오른쪽으로 한 칸씩 이동
            ra = (ra + 1) % n
        while visited[parent_b[lb] // NUM_COMBOS]: # 부모 b를 대상으로 첫 유전자의 왼쪽으로 한 칸씩
            lb = (lb - 1) % n
        while visited[parent_b[rb] // NUM_COMBOS]: # 부모 b를 대상으로 첫 유전자의 오른쪽으로 한 칸씩
            rb = (rb + 1) % n

        # 4개 포인터가 가리키는 위치의 cell ID만 뽑아서 후보 목록 생성 -> 총 4개의 셀 후보
        candidates = sorted({
            parent_a[la] // NUM_COMBOS, parent_a[ra] // NUM_COMBOS,
            parent_b[lb] // NUM_COMBOS, parent_b[rb] // NUM_COMBOS,
        })

        # 4개의 셀 후보 x 8개의 조합 = 32개 후보 중 최선 찾기
        # (조합마다 셀 내부 비용이 다르므로 전이 비용과 함께 비교)
        best = None
        for cand in candidates:
            for k in range(NUM_COMBOS):
                d = dist(prev_exit, cell_paths[cand].entries[k]) + cell_paths[cand].costs[k]
                if best is None or d < best[0]:
                    best = (d, cand * NUM_COMBOS + k)

        child.append(best[1])
        visited[best[1] // NUM_COMBOS] = True

    return child


def _swap_mutation(genes, rate, rng):
    if len(genes) >= 2 and rng.random() < rate: # 유전자가 최소 2개 있고, 난수를 뽑아서 설정한 확률값 미만이면 발동
        i, j = rng.sample(range(len(genes)), 2)
        genes[i], genes[j] = genes[j], genes[i]
    return genes


def _optimal_combos(genes, cell_paths, start_pos, dist):
    """논문에서 Algorithm 1(DP)를 구현한 부분"""
    seq = [g // NUM_COMBOS for g in genes] # 입력받은 genes에서 조합 정보는 버리고 cell 순서만 뽑아냄, 고정된 방문 순서는 안 바뀌고 각 cell의 조합만 새로 찾는 게 이 함수의 목표
    n = len(seq)
    inf = float("inf")

    dp = [[inf] * NUM_COMBOS for _ in range(n)] # 순서상 i번째 cell까지 지나왔고, i번째 cell을 조합 k로 빠져나왔을 때의 최소 누적 비용
    back = [[-1] * NUM_COMBOS for _ in range(n)] # 그 최소값이 이전 cell의 어떤 조합에서 왔는지를 기록

    # 첫 번째 cell의 4가지 조합 각각에 대해 로봇 시작점에서 그 조합의 entry까지거리
    # (조합마다 셀 내부 비용이 다를 수 있으므로 함께 더한다)
    for k in range(NUM_COMBOS):
        dp[0][k] = dist(start_pos, cell_paths[seq[0]].entries[k]) + cell_paths[seq[0]].costs[k]

    # 나머지 cell들 순서대로 DP 갱신
    for i in range(1, n):
        prev_exits = cell_paths[seq[i - 1]].exits
        cur_entries = cell_paths[seq[i]].entries
        cur_costs = cell_paths[seq[i]].costs
        for k in range(NUM_COMBOS):
            entry = cur_entries[k]
            best_pk, best_cost = -1, inf
            for pk in range(NUM_COMBOS):
                cost = dp[i - 1][pk] + dist(prev_exits[pk], entry)
                if cost < best_cost:
                    best_cost, best_pk = cost, pk
            dp[i][k] = best_cost + cur_costs[k]
            back[i][k] = best_pk

    # 마지막 cell에서 최선 찾기
    end_k = min(range(NUM_COMBOS), key=lambda k: dp[n - 1][k])
    choices = [0] * n
    choices[n - 1] = end_k
    for i in range(n - 1, 0, -1):
        choices[i - 1] = back[i][choices[i]]

    return [seq[i] * NUM_COMBOS + choices[i] for i in range(n)]


# ---------------------------------------------------------------------------
# 솔버
# ---------------------------------------------------------------------------

def solve_tsp_cpp(
    cells,
    start_pos,
    is_free=None,
    rows=None,
    cols=None,
    population_size=150,
    generations=250,
    crossover_rate=0.9,
    swap_mutation_rate=0.1,
    elite_size=4,
    patience=60,
    turn_penalty=0.01,
    seed=None,
):
    # 준비 단계
    # 거리 계산 함수를 준비
    # 각 cell의 8가지 조합(2방향 x 4코너) 정보를 미리 다 계산해둠
    # turn_penalty: 셀 내부 회전 1번당 추가 비용 (그리드 칸 단위). 작은 값이면
    # 길이가 같은 조합들 사이의 tie-breaker, 크게 하면 회전 수 자체를 줄이는 쪽으로 최적화
    n = len(cells)
    dist, _cache = _make_distance_fn(is_free, rows, cols)
    cell_paths = [CellPaths.build(c, dist, turn_penalty) for c in cells]

    # 예외 처리: cell이 0개 또는 1개일 때
    if n == 0:
        return [], [], [], {"best_length": 0.0, "generations_run": 0}

    rng = random.Random(seed)

    # cell이 1개 일 때는 GA를 돌릴 필요조차 없음
    if n == 1:
        best_k = min(range(NUM_COMBOS), key=lambda k: dist(start_pos, cell_paths[0].entries[k]) + cell_paths[0].costs[k])
        genes = [best_k]
        length = _genes_tour_length(genes, cell_paths, start_pos, dist)
        return [0], [best_k], cell_paths, {"best_length": length, "generations_run": 0}

    # cost 함수 정의
    def cost(genes):
        return _genes_tour_length(genes, cell_paths, start_pos, dist)

    # 초기 population: 무작위 순서 + 무작위 조합으로 만든 뒤, 각 개체의
    # 조합을 그 순서에 대해 DP로 최적화한다 (참조 코드가 매 세대 하는 것처럼).
    population = []
    for _ in range(population_size):
        genes = _optimal_combos(_random_genes(n, rng), cell_paths, start_pos, dist)
        population.append(genes)

    best = list(min(population, key=cost))
    best_cost = cost(best)
    stale = 0
    generations_run = 0

    # 메인 루프 : 세대 반복
    for gen in range(generations):
        generations_run = gen + 1
        costs = [cost(ind) for ind in population]

        # 최고 기록 갱신 + 조기 종료 체크 (논문에 없던 추가 기능)
        gen_best_idx = min(range(len(population)), key=lambda i: costs[i])
        if costs[gen_best_idx] < best_cost - 1e-12:
            best_cost = costs[gen_best_idx]
            best = list(population[gen_best_idx])
            stale = 0
        else:
            stale += 1
        if stale >= patience:
            break

        # Elitism: 상위 개체 그대로 보존
        ranked = sorted(range(len(population)), key=lambda i: costs[i])
        next_population = [list(population[i]) for i in ranked[:elite_size]]

        # 나머지 population 채우기 : crossover 또는 복사
        prefix = _roulette_prefix(costs) # 룰렛휠 확률표 준비
        while len(next_population) < population_size: # 90% 확률로 crossover: 룰렛휠로 부모 두 개 뽑아서 crossover 진행
            if rng.random() < crossover_rate:
                pa = population[_roulette_pick(prefix, rng)]
                pb = population[_roulette_pick(prefix, rng)]
                child = _heuristic_crossover(pa, pb, cell_paths, dist, rng)
            else: # 나머지 10% 확률로는 그냥 부모 하나를 그대로 복사
                child = list(population[_roulette_pick(prefix, rng)])

            # 확률적으로 순서 흔들기
            child = _swap_mutation(child, swap_mutation_rate, rng)
            # 흔들린 순서에 맞춰 조합 재최적화 DP
            child = _optimal_combos(child, cell_paths, start_pos, dist)
            next_population.append(child)

        population = next_population

    order = [g // NUM_COMBOS for g in best]
    combo = [0] * n
    for g in best:
        combo[g // NUM_COMBOS] = g % NUM_COMBOS

    stats = {"best_length": best_cost, "generations_run": generations_run}
    return order, combo, cell_paths, stats
