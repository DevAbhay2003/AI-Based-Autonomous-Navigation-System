import math
from typing import List, Tuple, Optional
from config.config import Config

class PIDController:
    """Proportional-Integral-Derivative controller for longitudinal speed tracking."""

    def __init__(self, kp: float = Config.KP, ki: float = Config.KI, kd: float = Config.KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, target_speed: float, current_speed: float, dt: float) -> float:
        """Computes commanded acceleration given target and current speed."""
        error = target_speed - current_speed
        self.integral += error * dt
        # Anti-windup clamping
        self.integral = max(-5.0, min(5.0, self.integral))
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error

        accel = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(-Config.MAX_BRAKING, min(Config.MAX_ACCELERATION, accel))

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

class PurePursuitController:
    """
    Geometric path-tracking controller that calculates steering angle based on
    a circular arc to a lookahead point along the planned trajectory.
    """

    def __init__(self, wheelbase: float = Config.WHEELBASE, lookahead_base: float = Config.LOOKAHEAD_DISTANCE):
        self.wheelbase = wheelbase
        self.lookahead_base = lookahead_base

    def find_lookahead_point(self, current_x: float, current_y: float, current_v: float,
                             path: List[Tuple[float, float]]) -> Tuple[Optional[Tuple[float, float]], int]:
        """Finds waypoint along trajectory that is at least lookahead distance ahead of current position."""
        if not path:
            return None, -1

        # 1. Find the index of the closest waypoint along the path
        closest_idx = 0
        min_dist_to_path = float("inf")
        for i, (wx, wy) in enumerate(path):
            d = math.hypot(wx - current_x, wy - current_y)
            if d < min_dist_to_path:
                min_dist_to_path = d
                closest_idx = i

        # 2. Dynamic lookahead distance proportional to velocity
        lookahead_dist = max(Config.MIN_LOOKAHEAD, self.lookahead_base + Config.LOOKAHEAD_GAIN * current_v)

        # 3. Search forward along path from closest_idx
        for i in range(closest_idx, len(path)):
            d = math.hypot(path[i][0] - current_x, path[i][1] - current_y)
            if d >= lookahead_dist:
                return path[i], i

        return path[-1], len(path) - 1

    def compute_steering(self, current_x: float, current_y: float, current_yaw: float,
                         current_v: float, path: List[Tuple[float, float]]) -> float:
        """
        Calculates steering angle:
        delta = atan2(2 * L * sin(alpha), L_d)
        where alpha is heading error to lookahead waypoint, and L_d is lookahead distance.
        """
        target_pt, _ = self.find_lookahead_point(current_x, current_y, current_v, path)
        if target_pt is None:
            return 0.0

        tx, ty = target_pt
        dx = tx - current_x
        dy = ty - current_y

        # Angle from vehicle to target point in world coordinates
        angle_to_target = math.atan2(dy, dx)
        # Relative heading error alpha normalized to [-pi, pi]
        alpha = angle_to_target - current_yaw
        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi

        # Distance to target lookahead point L_d
        ld = max(0.5, math.hypot(dx, dy))

        # Pure Pursuit curvature formula
        steer_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        return max(-Config.MAX_STEER_ANGLE, min(Config.MAX_STEER_ANGLE, steer_angle))
