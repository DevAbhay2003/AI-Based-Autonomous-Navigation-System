import math
import random
from typing import List, Tuple, Dict, Any
from config.config import Config

class LidarScanResult:
    """Contains raw telemetry produced by a LiDAR scan sweep."""

    def __init__(self, ranges: List[float], angles: List[float], hit_points: List[Tuple[float, float]]):
        self.ranges = ranges          # Radial distances (meters)
        self.angles = angles          # Relative beam angles (radians)
        self.hit_points = hit_points  # World coordinates of ray endpoints (meters)
        self.min_distance = min(ranges) if ranges else Config.LIDAR_MAX_RANGE

        # Calculate forward cone minimum distance (rays within +/- 45 degrees of front)
        forward_ranges = [
            r for r, ang in zip(ranges, angles) if abs(ang) <= math.radians(45)
        ]
        self.forward_min_distance = min(forward_ranges) if forward_ranges else Config.LIDAR_MAX_RANGE

class LidarSensor:
    """
    Simulates a 2D Planar LiDAR laser scanner with raycasting, range clipping,
    and Gaussian measurement noise.
    """

    def __init__(self, num_rays: int = Config.LIDAR_NUM_RAYS,
                 max_range: float = Config.LIDAR_MAX_RANGE,
                 noise_std: float = Config.LIDAR_NOISE_STD):
        self.num_rays = num_rays
        self.max_range = max_range
        self.noise_std = noise_std

        # Precompute relative angles evenly spaced across 360 degrees (-pi to pi)
        self.relative_angles = [
            -math.pi + (2.0 * math.pi * i / self.num_rays) for i in range(self.num_rays)
        ]

    def _ray_segment_intersection(self, p1: Tuple[float, float], p2: Tuple[float, float],
                                   p3: Tuple[float, float], p4: Tuple[float, float]) -> Tuple[bool, float]:
        """Calculates line segment intersection between (p1->p2) and (p3->p4)."""
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        x4, y4 = p4

        denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
        if abs(denom) < 1e-9:
            return False, 1.0  # Lines are parallel

        ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
        ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

        if 0.0 <= ua <= 1.0 and 0.0 <= ub <= 1.0:
            return True, ua

        return False, 1.0

    def scan(self, origin_x: float, origin_y: float, orientation_yaw: float,
             obstacle_segments: List[Tuple[Tuple[float, float], Tuple[float, float]]]) -> LidarScanResult:
        """
        Performs 360-degree raycasting sweep from vehicle origin against map obstacle segments.
        """
        ranges = []
        hit_points = []

        for rel_angle in self.relative_angles:
            global_angle = orientation_yaw + rel_angle
            ray_dx = math.cos(global_angle) * self.max_range
            ray_dy = math.sin(global_angle) * self.max_range
            target_pt = (origin_x + ray_dx, origin_y + ray_dy)

            closest_t = 1.0
            hit_detected = False

            for seg_start, seg_end in obstacle_segments:
                intersected, t = self._ray_segment_intersection(
                    (origin_x, origin_y), target_pt, seg_start, seg_end
                )
                if intersected and t < closest_t:
                    closest_t = t
                    hit_detected = True

            measured_range = closest_t * self.max_range

            # Inject realistic Gaussian measurement noise if hitting an object
            if hit_detected and self.noise_std > 0.0:
                noise = random.gauss(0.0, self.noise_std)
                measured_range = max(0.1, min(self.max_range, measured_range + noise))

            # Compute hit coordinate
            hx = origin_x + math.cos(global_angle) * measured_range
            hy = origin_y + math.sin(global_angle) * measured_range

            ranges.append(measured_range)
            hit_points.append((hx, hy))

        return LidarScanResult(ranges, self.relative_angles, hit_points)
