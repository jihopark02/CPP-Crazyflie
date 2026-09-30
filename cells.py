"""decomposition.py의 분해 결과와 TSP-CPP 플래너(tsp_cpp_ga.py)가 함께
쓰는 셀 자료구조."""

from dataclasses import dataclass, field


@dataclass
class Cell:
    """분해된 하나의 셀. gridpoints는 (row, col) 튜플 리스트."""
    id: int
    gridpoints: list
    centroid: tuple = field(init=False)

    def __post_init__(self):
        rs = sum(p[0] for p in self.gridpoints) / len(self.gridpoints)
        cs = sum(p[1] for p in self.gridpoints) / len(self.gridpoints)
        self.centroid = (rs, cs)


def cells_from_labels(labels, num_cells):
    """decomposition.decompose_trapezoidal()이 반환한 labels 그리드(장애물=-1,
    자유공간=cell id)로부터 Cell 목록(centroid + gridpoint 포함)을 만든다."""
    buckets = [[] for _ in range(num_cells)]
    for r, row in enumerate(labels):
        for c, label in enumerate(row):
            if label >= 0:
                buckets[label].append((r, c))
    return [Cell(id=i, gridpoints=pts) for i, pts in enumerate(buckets) if pts]
