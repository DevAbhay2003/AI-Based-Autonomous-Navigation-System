import math
import json
import os
from typing import List, Tuple, Dict, Any

class PerformanceMetrics:
    """Tracks and evaluates autonomous navigation benchmark metrics."""

    def __init__(self):
        self.trajectory: List[Tuple[float, float]] = []
        self.velocities: List[float] = []
        self.steering_angles: List[float] = []
        self.clearances: List[float] = []
        self.start_time: float = 0.0
        self.total_time: float = 0.0
        self.collision_count: int = 0
        self.goal_reached: bool = False

    def update(self, x: float, y: float, velocity: float, steer: float, min_clearance: float, dt: float):
        self.trajectory.append((x, y))
        self.velocities.append(velocity)
        self.steering_angles.append(steer)
        self.clearances.append(min_clearance)
        self.total_time += dt

    def register_collision(self):
        self.collision_count += 1

    def calculate_path_length(self) -> float:
        if len(self.trajectory) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.trajectory)):
            dx = self.trajectory[i][0] - self.trajectory[i-1][0]
            dy = self.trajectory[i][1] - self.trajectory[i-1][1]
            total += math.hypot(dx, dy)
        return total

    def calculate_smoothness(self) -> float:
        """Calculates mean square jerk/steering angle variation."""
        if len(self.steering_angles) < 2:
            return 0.0
        diffs = [abs(self.steering_angles[i] - self.steering_angles[i-1]) for i in range(1, len(self.steering_angles))]
        return sum(diffs) / len(diffs)

    def summary(self) -> Dict[str, Any]:
        path_len = self.calculate_path_length()
        avg_speed = (sum(self.velocities) / len(self.velocities)) if self.velocities else 0.0
        min_clear = min(self.clearances) if self.clearances else 0.0
        smoothness = self.calculate_smoothness()

        return {
            "mission_success": self.goal_reached,
            "total_time_sec": round(self.total_time, 2),
            "distance_traveled_m": round(path_len, 2),
            "average_speed_mps": round(avg_speed, 2),
            "average_speed_kmh": round(avg_speed * 3.6, 2),
            "min_obstacle_clearance_m": round(min_clear, 2),
            "total_collisions": self.collision_count,
            "steering_smoothness_index": round(smoothness, 4)
        }

    def save_to_file(self, filepath: str):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, "w") as f:
            json.dump(self.summary(), f, indent=4)
