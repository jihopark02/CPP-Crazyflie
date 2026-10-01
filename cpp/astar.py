"""그리드 A* 경로 탐색.

커버리지 경로에서 상하좌우로 바로 붙어 있지 않은 두 점(대부분 한 셀의
이탈점과 다음 셀의 진입점, 즉 agriculture 논문 용어로 "inter-region
path" -- grid_map.py 참고)을 이을 때 사용한다. 장애물이나 다른 셀을
가로지를 수 있는 직선 점프 대신, 장애물을 피해 가는 실제 그리드 이동
경로로 연결한다.
"""

import heapq


def _heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def find_path(is_free, rows, cols, start, goal):
    """subcell 그리드 위에서 4방향(상하좌우) A* 탐색. `is_free(r, c)`는
    맵 안에 있고 장애물이 아닌 subcell이면 True를 반환해야 한다.
    `start`부터 `goal`까지(양 끝 포함) (row, col) 점 리스트를 반환하며,
    각 점은 직전 점과 상하좌우로 한 칸 차이다. 도달할 수 없으면 None."""
    if start == goal:
        return [start]
    if not is_free(*start) or not is_free(*goal):
        return None

    open_heap = [(_heuristic(start, goal), 0, start)]
    came_from = {}
    best_g = {start: 0}
    closed = set()

    while open_heap:
        _, g, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while path[-1] != start:
                path.append(came_from[path[-1]])
            path.reverse()
            return path
        closed.add(current)

        r, c = current
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbor = (r + dr, c + dc)
            if not (0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols):
                continue
            if not is_free(*neighbor):
                continue
            if neighbor in closed:
                continue
            ng = g + 1
            if ng < best_g.get(neighbor, float("inf")):
                best_g[neighbor] = ng
                came_from[neighbor] = current
                heapq.heappush(open_heap, (ng + _heuristic(neighbor, goal), ng, neighbor))

    return None


def stitch_path(is_free, rows, cols, points):
    """`points`를 따라가면서, 연속한 두 점이 한 칸 이동이 아닌 곳마다
    A*로 구한 우회 경로를 끼워 넣는다. 그 결과 모든 연속한 두 점이
    정확히 한 칸 차이인 새 리스트를 반환한다. 연속으로 중복된 점은
    하나로 합친다. A*로 이을 수 없는 점(서로 끊어진 영역)은 그냥 바로
    점프하도록 둔다 -- 애초에 도달 가능한 셀들이라 일어나면 안 되는
    일이지만, 그렇다고 프로그램을 멈출 정도는 아니다.

    반환값: (stitched_points, detours). `detours`는 A*가 필요했던 모든
    구간의 (from_point, to_point, detour_length) 리스트.
    """
    if not points:
        return [], []

    stitched = [points[0]]
    detours = []

    for i in range(1, len(points)):
        prev = stitched[-1]
        cur = points[i]
        dist = abs(cur[0] - prev[0]) + abs(cur[1] - prev[1])
        if dist == 0:
            continue
        if dist == 1:
            stitched.append(cur)
            continue

        sub = find_path(is_free, rows, cols, prev, cur)
        if sub is None:
            stitched.append(cur)  # 도달 불가 -- 바로 점프로 대체
        else:
            stitched.extend(sub[1:])
            detours.append((prev, cur, len(sub) - 1))

    return stitched, detours
