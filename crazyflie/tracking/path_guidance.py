"""Platform-independent coverage-path guidance."""

import bisect
import math


def _clamp(value, low, high):
    return max(low, min(high, value))


class PathGuidance:
    """Monotonic path progress with segment-projection guidance."""

    def __init__(self, path, dist, lookahead_distance, forward_search_distance):
        if len(path) < 2 or len(path) != len(dist):
            raise ValueError("path and dist must contain the same two or more points")

        self.path = path
        self.dist = dist
        self.lookahead_distance = lookahead_distance
        self.forward_search_distance = forward_search_distance
        self.n = len(path)
        self.path_idx = 0
        self.progress_s = dist[0]

    @staticmethod
    def _project_segment(x, y, ax, ay, bx, by):
        abx, aby = bx - ax, by - ay
        length_sq = abx * abx + aby * aby
        if length_sq < 1e-12:
            return ax, ay, 0.0, math.hypot(x - ax, y - ay)
        u = _clamp(((x - ax) * abx + (y - ay) * aby) / length_sq, 0.0, 1.0)
        qx, qy = ax + u * abx, ay + u * aby
        return qx, qy, u, math.hypot(x - qx, y - qy)

    def _point_at_s(self, s):
        s = _clamp(s, self.dist[0], self.dist[-1])
        i = _clamp(bisect.bisect_right(self.dist, s) - 1, 0, self.n - 2)
        ds = self.dist[i + 1] - self.dist[i]
        u = 0.0 if ds <= 1e-12 else (s - self.dist[i]) / ds
        ax, ay = self.path[i]
        bx, by = self.path[i + 1]
        return ax + u * (bx - ax), ay + u * (by - ay), i

    def update(self, x, y):
        hi_s = self.progress_s + self.forward_search_distance
        end = self.path_idx
        while end < self.n - 2 and self.dist[end + 1] <= hi_s:
            end += 1

        best = None
        for i in range(self.path_idx, end + 1):
            ax, ay = self.path[i]
            bx, by = self.path[i + 1]
            qx, qy, u, distance = self._project_segment(x, y, ax, ay, bx, by)
            segment_ds = self.dist[i + 1] - self.dist[i]
            projected_s = self.dist[i] + u * segment_ds
            if projected_s + 1e-9 < self.progress_s:
                continue
            if best is None or distance < best[0]:
                best = (distance, i, projected_s, qx, qy)

        if best is None:
            qx, qy, i = self._point_at_s(self.progress_s)
        else:
            _, i, projected_s, qx, qy = best
            self.progress_s = max(self.progress_s, projected_s)
            self.path_idx = i

        target_s = min(self.progress_s + self.lookahead_distance, self.dist[-1])
        target_x, target_y, _ = self._point_at_s(target_s)

        tx, ty = target_x - qx, target_y - qy
        tangent_norm = math.hypot(tx, ty)
        if tangent_norm < 1e-9:
            ax, ay = self.path[self.path_idx]
            bx, by = self.path[min(self.path_idx + 1, self.n - 1)]
            tx, ty = bx - ax, by - ay
            tangent_norm = max(math.hypot(tx, ty), 1e-9)
        tx, ty = tx / tangent_norm, ty / tangent_norm

        ax, ay = self.path[self.path_idx]
        bx, by = self.path[min(self.path_idx + 1, self.n - 1)]
        sx, sy = bx - ax, by - ay
        segment_norm = max(math.hypot(sx, sy), 1e-9)
        sx, sy = sx / segment_norm, sy / segment_norm
        turn_angle = abs(math.atan2(sx * ty - sy * tx, sx * tx + sy * ty))

        return qx, qy, target_x, target_y, tx, ty, turn_angle
