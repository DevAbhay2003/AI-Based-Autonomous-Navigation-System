import math
from typing import Tuple

class VehicleState:
    """Represents the instantaneous kinematic state of the autonomous vehicle."""

    def __init__(self, x: float = 5.0, y: float = 18.0, yaw: float = 0.0, v: float = 0.0):
        self.x = x          # meters
        self.y = y          # meters
        self.yaw = yaw      # radians
        self.v = v          # m/s (linear speed)
        self.steer = 0.0    # radians (front wheel steer)

class BicycleVehicleModel:
    """
    Standard Kinematic Bicycle Model for non-holonomic wheeled mobile robots.
    Simulates motion under non-slip constraints based on rear-axle kinematics.
    """

    def __init__(self, wheelbase: float = 1.8, max_steer: float = 0.65, max_speed: float = 6.0):
        self.wheelbase = wheelbase
        self.max_steer = max_steer
        self.max_speed = max_speed
        self.state = VehicleState()

    def set_pose(self, x: float, y: float, yaw: float, v: float = 0.0):
        """Sets vehicle state pose."""
        self.state.x = x
        self.state.y = y
        self.state.yaw = yaw
        self.state.v = v
        self.state.steer = 0.0

    def update(self, throttle_accel: float, target_steer: float, dt: float):
        """
        Updates kinematic state via Euler numerical integration over time step dt.
        
        Args:
            throttle_accel: Longitudinal acceleration (m/s^2, positive=accel, negative=brake)
            target_steer: Commanded steering angle (radians)
            dt: Delta time interval (seconds)
        """
        # Clamp steering angle within mechanical physical limits
        self.state.steer = max(-self.max_steer, min(self.max_steer, target_steer))

        # Update speed with acceleration limit
        self.state.v += throttle_accel * dt
        self.state.v = max(0.0, min(self.max_speed, self.state.v))

        # Kinematic bicycle equations
        self.state.x += self.state.v * math.cos(self.state.yaw) * dt
        self.state.y += self.state.v * math.sin(self.state.yaw) * dt
        self.state.yaw += (self.state.v / self.wheelbase) * math.tan(self.state.steer) * dt

        # Normalize yaw angle to [-pi, pi]
        self.state.yaw = (self.state.yaw + math.pi) % (2.0 * math.pi) - math.pi

    def get_bounding_box(self, length: float = 2.4, width: float = 1.2) -> Tuple[Tuple[float, float], ...]:
        """Returns the 4 corner coordinates of the vehicle oriented bounding box."""
        half_l = length / 2.0
        half_w = width / 2.0
        cos_yaw = math.cos(self.state.yaw)
        sin_yaw = math.sin(self.state.yaw)

        corners_rel = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, -half_w),
            (-half_l, half_w)
        ]

        corners_world = []
        for rx, ry in corners_rel:
            wx = self.state.x + (rx * cos_yaw - ry * sin_yaw)
            wy = self.state.y + (rx * sin_yaw + ry * cos_yaw)
            corners_world.append((wx, wy))

        return tuple(corners_world)
