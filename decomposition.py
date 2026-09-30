import colorsys


def _free_intervals(column):
    """column: bool 리스트 (True=장애물). 장애물이 아닌 셀들의 연속 구간(row_start, row_end)을 반환."""
    intervals = []
    start = None
    for r, is_obstacle in enumerate(column):
        if not is_obstacle:
            if start is None:
                start = r
        else:
            if start is not None:
                intervals.append((start, r - 1))
                start = None
    if start is not None:
        intervals.append((start, len(column) - 1))
    return intervals


def _overlaps(a, b):
    return a[0] <= b[1] and b[0] <= a[1]


def decompose_trapezoidal(obstacles, rows, cols):
    """
    그리드 맵에 대한 사다리꼴(trapezoidal / Boustrophedon) 셀 분해.

    좌->우로 컬럼을 스윕하면서 각 컬럼의 장애물 없는 연속 구간(free interval)을 구하고,
    인접한 두 컬럼의 구간들을 행 범위 겹침으로 연결한다. 연결 관계가 단순한
    1:1 대응이 아닌 지점(구간이 새로 나타남/사라짐/분리/병합)마다 새로운 셀을 시작한다.
    이는 다각형 기반 사다리꼴 분해에서 장애물 꼭짓점을 지날 때 셀이 갈라지는 것과
    동일한 원리를 그리드 상에서 구현한 것이다.

    Returns:
        labels: rows x cols 크기의 2D 리스트. 장애물은 -1, 자유 공간은 셀 id(0..num_cells-1).
        num_cells: 생성된 셀 개수.
    """
    labels = [[-1 for _ in range(cols)] for _ in range(rows)]
    next_id = 0
    prev_intervals = []  # (구간, cell_id) 튜플 리스트

    for c in range(cols):
        column = [obstacles[r][c] for r in range(rows)]
        intervals = _free_intervals(column)

        matches = [
            [j for j, (pinterval, _pid) in enumerate(prev_intervals) if _overlaps(interval, pinterval)]
            for interval in intervals
        ]
        prev_matches = [[] for _ in prev_intervals]
        for i, js in enumerate(matches):
            for j in js:
                prev_matches[j].append(i)

        new_prev_intervals = []
        for i, interval in enumerate(intervals):
            js = matches[i]
            if len(js) == 1 and len(prev_matches[js[0]]) == 1:
                cell_id = prev_intervals[js[0]][1]
            else:
                cell_id = next_id
                next_id += 1

            for r in range(interval[0], interval[1] + 1):
                labels[r][c] = cell_id
            new_prev_intervals.append((interval, cell_id))

        prev_intervals = new_prev_intervals

    return labels, next_id


def generate_palette(n):
    """n개의 서로 잘 구분되는 RGB 색상을 생성 (황금비 간격의 hue 회전)."""
    colors = []
    for i in range(max(n, 1)):
        hue = (i * 0.61803398875) % 1.0
        r, g, b = colorsys.hsv_to_rgb(hue, 0.30, 0.97)  # 파스텔 톤: 경로/마커가 잘 보이게
        colors.append((int(r * 255), int(g * 255), int(b * 255)))
    return colors
