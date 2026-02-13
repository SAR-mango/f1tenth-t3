import numpy as np
from math import atan2
from collections import namedtuple

Point = namedtuple("Point", ["x", "y"])


class Circle:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

    def create_array(self, start_angle, end_angle, sample_count=50):
        points = np.zeros((sample_count, 2))
        angles = np.linspace(start_angle, end_angle, sample_count)
        points[:, 0] = self.center.x + np.sin(angles) * self.radius
        points[:, 1] = self.center.y + np.cos(angles) * self.radius
        return points

    def get_angle(self, point):
        return atan2(point.x - self.center.x, point.y - self.center.y)

    def get_closest_point(self, point):
        x = point.x - self.center.x
        y = point.y - self.center.y
        distance = (x ** 2 + y ** 2) ** 0.5
        return Point(
            self.center.x + x * self.radius / distance,
            self.center.y + y * self.radius / distance,
        )

    @staticmethod
    def fit(points):
        if points is None or points.shape[0] < 3:
            raise ValueError("Need at least 3 points to fit a circle.")

        x = points[:, 0]
        y = points[:, 1]

        # Least-squares circle fit:
        # (x-cx)^2 + (y-cy)^2 = r^2
        # -> 2*cx*x + 2*cy*y + c = x^2 + y^2
        a = np.column_stack((2.0 * x, 2.0 * y, np.ones(points.shape[0])))
        b = x * x + y * y

        solution, _, rank, _ = np.linalg.lstsq(a, b, rcond=None)
        if rank < 3:
            raise ValueError("Circle fit is rank-deficient.")

        center_x = float(solution[0])
        center_y = float(solution[1])
        c = float(solution[2])
        radius_sq = center_x * center_x + center_y * center_y + c
        if not np.isfinite(radius_sq) or radius_sq <= 0.0:
            raise ValueError("Computed invalid circle radius.")

        radius = float(np.sqrt(radius_sq))
        return Circle(Point(center_x, center_y), radius)
