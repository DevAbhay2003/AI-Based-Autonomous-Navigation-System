import math
from typing import List, Tuple, Dict, Any
from enum import Enum
from config.config import Config
from src.control.pid_controller import PurePursuitController, PIDController

class NavigationState(Enum):
    CRUISING = "CRUISING"
    AVOIDING = "AVOIDING_OBSTACLE"
    EMERGENCY_BRAKE = "EMERGENCY_BRAKE"
    GOAL_REACHED = "GOAL_REACHED"

class LocalPlanner:
    """
    Real-time local reactive planner integrating Pure Pursuit waypoint guidance,
    reactive repulsive vector fields for dynamic obstacle evasion, and a safety FSM.
    """

    def __init__(self):
        self.state = NavigationState.CRUISING
        self.pure_pursuit = PurePursuitController()
        self.pid_speed = PIDController()
        self.goal_tolerance = 1.2 # meters

    def update(self, current_x: float, current_y: float, current_yaw: float, current_v: float,
               global_path: List[Tuple[float, float]],
               min_lidar_dist: float,
               dynamic_obstacles: List[Dict[str, Any]],
               dt: float) -> Tuple[float, float, NavigationState]:
        """
        Computes the target throttle acceleration and steering angle for the current timestep.
        
        Returns:
            throttle_accel: acceleration command (m/s^2)
            steer_angle: steering command (radians)
            state: current NavigationState
        """
        # 1. Check goal condition
        if global_path:
            gx, gy = global_path[-1]
            dist_to_goal = math.hypot(gx - current_x, gy - current_y)
            if dist_to_goal <= self.goal_tolerance:
                self.state = NavigationState.GOAL_REACHED
                brake = self.pid_speed.compute(target_speed=0.0, current_speed=current_v, dt=dt)
                return brake, 0.0, self.state

        # 2. Base steering from global path Pure Pursuit
        base_steer = self.pure_pursuit.compute_steering(
            current_x, current_y, current_yaw, current_v, global_path
        )

        # 3. Dynamic Obstacle Reactive Repulsion
        repulsive_steer_offset = 0.0
        repulsion_weight = 0.8
        detected_danger = False

        emergency_dynamic_stop = False
        for obs in dynamic_obstacles:
            ox, oy = obs["x"], obs["y"]
            dx = ox - current_x
            dy = oy - current_y
            dist = math.hypot(dx, dy)

            # Check if dynamic obstacle is within active collision horizon
            if dist < 8.0:
                angle_to_obs = math.atan2(dy, dx)
                rel_angle = angle_to_obs - current_yaw
                rel_angle = (rel_angle + math.pi) % (2.0 * math.pi) - math.pi

                # Obstacle is within forward semi-circle (-75 to +75 degrees)
                if abs(rel_angle) < math.radians(75) or dist < 3.5:
                    detected_danger = True
                    if dist < 4.0:
                        emergency_dynamic_stop = True
                    # Repulse in opposite lateral direction
                    repulsion_sign = -1.0 if rel_angle > 0 else 1.0
                    intensity = (8.0 - dist) / 8.0
                    repulsive_steer_offset += repulsion_sign * intensity * repulsion_weight

        # 4. State Machine Transition & Velocity Profiling (Dynamic Braking Margin)
        stopping_distance = (current_v ** 2) / (2.0 * Config.MAX_BRAKING) + 1.2
        critical_threshold = max(2.2, stopping_distance)
        caution_threshold = critical_threshold + 3.0

        if emergency_dynamic_stop or min_lidar_dist < critical_threshold:
            self.state = NavigationState.EMERGENCY_BRAKE
            target_speed = 0.0
        elif detected_danger or min_lidar_dist < caution_threshold:
            self.state = NavigationState.AVOIDING
            clearance_ratio = max(0.1, min(1.0, (min_lidar_dist - critical_threshold) / 3.0))
            target_speed = Config.MAX_SPEED * 0.45 * clearance_ratio
        else:
            self.state = NavigationState.CRUISING
            target_speed = Config.MAX_SPEED

        # 5. Final Steering Angle Combination
        total_steer = base_steer + repulsive_steer_offset
        total_steer = max(-Config.MAX_STEER_ANGLE, min(Config.MAX_STEER_ANGLE, total_steer))

        # 6. Longitudinal Speed PID
        throttle_accel = self.pid_speed.compute(target_speed=target_speed, current_speed=current_v, dt=dt)

        return throttle_accel, total_steer, self.state
