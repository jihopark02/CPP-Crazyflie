"""곡률 제약 경로 스무딩.

Höffmann, Patel & Büskens, "Optimal Coverage Path Planning for
Agricultural Vehicles with Curvature Constraints" (Agriculture 2023, 13,
2112)의 "smooth path planning" 단계를 우리의
그리드 기반 커버리지 경로에 맞게 옮긴 것이다.

논문은 가이던스 트랙 사이의 급회전을 최적 제어(TransWORHP/WORHP)로 구한
연속 곡률(CC Dubins) 회전으로 만들고, 셀 사이를 잇는 꺾은선 경로는 가중치를
최적화한 NURBS 곡선(비선형 계획 문제로 풂)으로 스무딩한다. 두 방법 모두
최대 곡률 kappa_max = 1 / r_min 을 강제한다. 여기서는 그 솔버들(WORHP,
Gurobi)을 쓸 수 없으므로, 같은 *목표* -- 곡률이 절대 1 / r_min 을 넘지 않고
원래 꺾은선을 가깝게 따라가는 경로 -- 를 원호 모서리 둥글리기("필렛")로
구현한다: 원래 그리드 경로의 모든 모서리를 반지름 r_min 인 접선 원호로
바꾸고 (r_min 이 들어가지 않으면 모서리와 이웃 점 사이에 들어가는 가장 큰
반지름을 쓴다), 직선 구간은 그대로 둔다. 이는 논문의 CC Dubins / NURBS
최적화를 기하학적으로 정확한 닫힌 형태(closed-form)로 대신하는 방법이다:
모든 곳에서 곡률 <= 1/r_min 과 모든 연결점에서의 G1(접선) 연속성을
보장하지만, 논문의 C2 연속 곡률 전이는 얻지 못한다 (CC Dubins 모델처럼
클로소이드로 곡률이 부드럽게 올라가는 대신, 직선 구간의 곡률 0 에서 원호의
1/r 로 곡률이 계단식으로 바뀐다).

180도에 가까운 반전(그리드 경로에서는 드물지만 가능)은 비현실적으로 긴
직선 진입 구간 없이는 이 방식으로 둥글릴 수 없으므로, 반지름이 거의 0 인
뾰족한 첨점을 만드는 대신 둥글리지 않은 날카로운 꼭짓점으로 남겨 둔다 --
아래 MIN_USEFUL_RADIUS_RATIO 참고.
"""

import math
from dataclasses import dataclass, field


@dataclass
class CornerInfo:
    """둥글린 모서리 하나에 대한 진단 정보. 스무딩된 경로가 요청한 회전
    반지름에 얼마나 가까이 도달했는지 보고하는 데 쓴다."""
    index: int  # 원래 (중복 제거된) 경로에서 이 모서리의 인덱스
    turn_angle_deg: float
    requested_radius: float
    achieved_radius: float
    limited_by_segment_length: bool
    unrounded: bool = False  # True: 너무 급해서 둥글리지 못하고 날카로운 꼭짓점으로 남김


@dataclass
class SmoothPath:
    points: list = field(default_factory=list)      # 촘촘한 (x, y) 꺾은선
    cumulative_length: list = field(default_factory=list)  # points와 같은 길이
    corners: list = field(default_factory=list)      # list[CornerInfo]

    @property
    def total_length(self):
        return self.cumulative_length[-1] if self.cumulative_length else 0.0

    def point_at_distance(self, distance):
        """경로를 따라 `distance` 미터 지점의 (x, y) 점을 반환한다
        ([0, total_length] 범위로 제한). 가장 가까운 두 샘플 점 사이를
        선형 보간한다."""
        if not self.points:
            return None
        if distance <= 0:
            return self.points[0]
        if distance >= self.total_length:
            return self.points[-1]

        lengths = self.cumulative_length
        lo, hi = 0, len(lengths) - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if lengths[mid] < distance:
                lo = mid + 1
            else:
                hi = mid
        i = max(lo, 1)
        seg_len = lengths[i] - lengths[i - 1]
        t = 0.0 if seg_len < 1e-12 else (distance - lengths[i - 1]) / seg_len
        (x0, y0), (x1, y1) = self.points[i - 1], self.points[i]
        return (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)


def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def dedupe_points(points):
    out = []
    for p in points:
        if not out or _dist(out[-1], p) > 1e-9:
            out.append(p)
    return out


MIN_USEFUL_RADIUS_RATIO = 0.02  # 달성 반지름이 요청 반지름의 이 비율보다 작으면 모서리를 날카롭게 둔다


def _fillet(p_prev, p, p_next, radius):
    """`p`의 모서리를 선분 (p_prev, p)와 (p, p_next)에 접하는 원호로
    둥글린다. 모서리가 (거의) 직선이라 둥글릴 필요가 없으면 None을,
    아니면 접점, 원호 중심/반지름/각도, 회전 방향을 담은 dict를 반환한다.

    180도에 가까운 반전은 양쪽에 아주 긴 직선 진입 구간 없이는 접선-원호-접선
    필렛 하나로 둥글릴 수 없다 (회전각 -> 180도 이면 tan(반각) -> 0 이라,
    접선 길이가 고정되어 있으면 달성 반지름이 0 으로 사라진다 -- 부드러운
    곡선이 아니라 반지름이 거의 0 인 첨점이 된다). 그런 첨점을 만들면
    국소적으로 곡률 제한을 크게 어기게 되므로, 대신 그 모서리는 날카롭게
    두고 unrounded로 보고한다. 실제 차량이라면 그 지점에서 별도의 U턴
    기동이 필요하다 -- 헤드랜드의 헤어핀 회전은 일반적인 모서리 필렛이 아닌
    전용 회전 유형이 필요하다는 논문의 지적(Figure 8)과 같은 얘기다.
    """
    v_in = (p[0] - p_prev[0], p[1] - p_prev[1])
    v_out = (p_next[0] - p[0], p_next[1] - p[1])
    len_in, len_out = math.hypot(*v_in), math.hypot(*v_out)
    if len_in < 1e-9 or len_out < 1e-9:
        return None
    u_in = (v_in[0] / len_in, v_in[1] / len_in)
    u_out = (v_out[0] / len_out, v_out[1] / len_out)

    cos_turn = max(-1.0, min(1.0, u_in[0] * u_out[0] + u_in[1] * u_out[1]))
    turn = math.acos(cos_turn)
    if turn < 1e-4:
        return None  # 일직선이라 둥글릴 필요 없음

    half = (math.pi - turn) / 2.0
    half = max(half, 1e-6)  # U턴에 가까울 때 tan()/sin()이 폭주하지 않게 방지
    tan_half = math.tan(half)

    max_tangent = 0.5 * min(len_in, len_out)
    desired_tangent = radius / tan_half
    tangent_len = min(desired_tangent, max_tangent)
    achieved_radius = tangent_len * tan_half
    limited = tangent_len < desired_tangent - 1e-9

    if achieved_radius < radius * MIN_USEFUL_RADIUS_RATIO:
        return {"unroundable": True, "turn_angle": turn}

    tangent_in = (p[0] - u_in[0] * tangent_len, p[1] - u_in[1] * tangent_len)
    tangent_out = (p[0] + u_out[0] * tangent_len, p[1] + u_out[1] * tangent_len)

    cross = u_in[0] * u_out[1] - u_in[1] * u_out[0]
    turn_sign = 1.0 if cross >= 0 else -1.0  # +1 = 좌회전/반시계, -1 = 우회전/시계

    def perp_left(u):
        return (-u[1], u[0])

    pl_in = perp_left(u_in)
    center = (
        tangent_in[0] + turn_sign * achieved_radius * pl_in[0],
        tangent_in[1] + turn_sign * achieved_radius * pl_in[1],
    )

    a0 = math.atan2(tangent_in[1] - center[1], tangent_in[0] - center[0])
    a1 = math.atan2(tangent_out[1] - center[1], tangent_out[0] - center[0])

    return {
        "tangent_in": tangent_in,
        "tangent_out": tangent_out,
        "center": center,
        "radius": achieved_radius,
        "a0": a0,
        "a1": a1,
        "turn_sign": turn_sign,
        "turn_angle": turn,
        "limited": limited,
    }


def _arc_points(fillet, samples_per_arc):
    a0, a1, sign = fillet["a0"], fillet["a1"], fillet["turn_sign"]
    # 회전 방향에 맞춰 a0에서 a1까지 짧은 쪽으로 쓸고 지나간다.
    if sign >= 0:  # 반시계: a1 > a0 이어야 함
        while a1 < a0:
            a1 += 2 * math.pi
    else:  # 시계: a1 < a0 이어야 함
        while a1 > a0:
            a1 -= 2 * math.pi

    n = max(2, samples_per_arc)
    cx, cy = fillet["center"]
    r = fillet["radius"]
    pts = []
    for i in range(n + 1):
        t = a0 + (a1 - a0) * i / n
        pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    return pts


def resample_by_distance(smooth_path, spacing):
    """`smooth_path`를 `spacing` 미터의 일정한 호 길이 간격으로 다시
    샘플링한다 (예: 로봇 제어기에 넘길 웨이포인트 내보내기용). 경로의
    첫 점과 마지막 점은 항상 포함해서, 리샘플링된 꺾은선도 스무딩된
    경로와 정확히 같은 곳에서 시작하고 끝나게 한다.

    (x, y, dist_along_path) 튜플 리스트를 반환한다. `dist_along_path`는
    각 점의 실제 호 길이 위치다 -- 마지막 간격은 보통 spacing 한 칸보다
    짧으므로, 마지막 점의 거리는 spacing의 배수가 아니라 경로의 실제 전체
    길이다.
    """
    pts = smooth_path.points
    total = smooth_path.total_length
    if not pts:
        return []
    if spacing <= 0 or total <= 0:
        return [(x, y, 0.0) for x, y in pts]

    out = []
    d = 0.0
    while d < total:
        x, y = smooth_path.point_at_distance(d)
        out.append((x, y, d))
        d += spacing

    last = pts[-1]
    if not out or _dist(out[-1][:2], last) > 1e-9:
        out.append((last[0], last[1], total))
    return out


def smooth_path(points, radius, samples_per_arc=10):
    """`points`((x, y) 튜플 리스트, 예: 미터처럼 일관된 실제 단위)의 모든
    모서리를 주어진 `radius`(또는 들어갈 수 있는 가장 큰 반지름 중 작은
    쪽)의 원호로 둥글리고, 직선 구간은 그대로 둔다.

    촘촘한 출력 꺾은선, 그 누적 호 길이(일정 속도 애니메이션용), 모서리별
    진단 정보를 담은 SmoothPath를 반환한다.
    """
    pts = dedupe_points(points)
    if len(pts) < 3 or radius <= 0:
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + _dist(pts[i - 1], pts[i]))
        return SmoothPath(points=pts, cumulative_length=cum, corners=[])

    out = [pts[0]]
    corners = []

    for i in range(1, len(pts) - 1):
        fillet = _fillet(pts[i - 1], pts[i], pts[i + 1], radius)
        if fillet is None:
            continue
        if fillet.get("unroundable"):
            out.append(pts[i])  # 날카로운 꼭짓점을 그대로 둠
            corners.append(CornerInfo(
                index=i,
                turn_angle_deg=math.degrees(fillet["turn_angle"]),
                requested_radius=radius,
                achieved_radius=0.0,
                limited_by_segment_length=True,
                unrounded=True,
            ))
            continue
        out.append(fillet["tangent_in"])
        out.extend(_arc_points(fillet, samples_per_arc)[1:-1])
        out.append(fillet["tangent_out"])
        corners.append(CornerInfo(
            index=i,
            turn_angle_deg=math.degrees(fillet["turn_angle"]),
            requested_radius=radius,
            achieved_radius=fillet["radius"],
            limited_by_segment_length=fillet["limited"],
        ))

    out.append(pts[-1])
    out = dedupe_points(out)

    cum = [0.0]
    for i in range(1, len(out)):
        cum.append(cum[-1] + _dist(out[i - 1], out[i]))

    return SmoothPath(points=out, cumulative_length=cum, corners=corners)
