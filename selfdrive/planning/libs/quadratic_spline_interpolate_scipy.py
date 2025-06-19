import numpy as np
import math
from scipy.interpolate import splrep, splev

class QuadraticSplineInterpolateFast:
    # ────────────────────────────────────────────────
    # 초기화: 스플라인 한 번만 구축
    # ────────────────────────────────────────────────
    def __init__(self, xs, ys):
        self.xs = np.asarray(xs, dtype=float)
        self.ys = np.asarray(ys, dtype=float)

        # 누적거리 s 축
        ds = np.hypot(np.diff(self.xs), np.diff(self.ys))
        self.s = np.hstack(([0.0], np.cumsum(ds)))

        # cubic B-spline 계수 (k=3, s=0 → 정확히 통과)
        self.tck_x = splrep(self.s, self.xs, k=3, s=0)
        self.tck_y = splrep(self.s, self.ys, k=3, s=0)

        # 속도 미리 캐싱: 전체 길이
        self.s_end = float(self.s[-1])

    def _derivatives(self, s_query, order):
        """order=1 → (x', y'), order=2 → (x'', y'')"""
        dx = splev(s_query, self.tck_x, der=order)
        dy = splev(s_query, self.tck_y, der=order)
        return dx, dy

    def yaw_and_curvature(self, s_query):
        """
        s_query : 1-D ndarray or list
        반환     : (yaw 배열[rad], curvature 배열[1/m])
        """
        s_query = np.asarray(s_query, dtype=float)

        x1, y1 = self._derivatives(s_query, 1)   # 1차
        x2, y2 = self._derivatives(s_query, 2)   # 2차

        yaw = np.arctan2(y1, x1)

        denom = np.power(x1 * x1 + y1 * y1, 1.5)
        k = (y2 * x1 - x2 * y1) / denom
        return yaw, k