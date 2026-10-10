import math
import random
from typing import List, Tuple, Dict, Any
from config.config import Config

class DynamicObstacle:
    """Represents a moving agent (pedestrian or other vehicle) in the environment."""

    def __init__(self, x: float, y: float, vx: float, vy: float,
                 width: float = 1.0, length: float = 1.0, obs_type: str = "Pedestrian"):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.width = width
        self.length = length
        self.obs_type = obs_type
        self.base_min_x = max(2.0, x - 12.0)
        self.base_max_x = min(Config.WORLD_WIDTH_M - 2.0, x + 12.0)

    def update(self, dt: float):
        self.x += self.vx * dt
        self.y += self.vy * dt

        # Bounce or reverse patrol trajectory along active velocity axis
        if self.vx != 0.0:
            if self.x <= self.base_min_x or self.x >= self.base_max_x:
                self.vx = -self.vx
        if self.vy != 0.0:
            if self.y <= self.base_min_x or self.y >= self.base_max_x:
                self.vy = -self.vy

    def get_bounding_box(self) -> Tuple[float, float, float, float]:
        """Returns (x, y, w, h) bounding box."""
        return (self.x - self.width / 2.0, self.y - self.length / 2.0, self.width, self.length)

class SimulationEnvironment:
    """
    Manages the virtual simulation world, static obstacles (buildings, walls),
    and dynamic obstacles (patrolling pedestrians and vehicles).
    """

    def __init__(self):
        self.static_obstacles: List[Tuple[float, float, float, float]] = [] # (x, y, w, h)
        self.dynamic_obstacles: List[DynamicObstacle] = []
        self.goal_pos: Tuple[float, float] = (58.0, 18.0)
        self.start_pos: Tuple[float, float] = (6.0, 18.0)
        self.setup_default_map()

    def setup_default_map(self):
        """Builds a realistic urban roadway navigation environment with obstacle clusters."""
        self.static_obstacles.clear()
        self.dynamic_obstacles.clear()

        # Outer boundary walls
        w = Config.WORLD_WIDTH_M
        h = Config.WORLD_HEIGHT_M

        # Boundary perimeter
        self.static_obstacles.append((0.0, 0.0, w, 1.0))         # Top wall
        self.static_obstacles.append((0.0, h - 1.0, w, 1.0))     # Bottom wall
        self.static_obstacles.append((0.0, 0.0, 1.0, h))         # Left wall
        self.static_obstacles.append((w - 1.0, 0.0, 1.0, h))     # Right wall

        # Urban block obstacles (buildings and barriers)
        # Block 1 (Top-mid barrier)
        self.static_obstacles.append((16.0, 8.0, 5.0, 8.0))
        # Block 2 (Bottom-mid barrier)
        self.static_obstacles.append((18.0, 22.0, 6.0, 7.0))
        # Central chicane obstacle
        self.static_obstacles.append((32.0, 14.0, 5.0, 8.0))
        # Block 4 (Top right building)
        self.static_obstacles.append((44.0, 6.0, 6.0, 9.0))
        # Block 5 (Bottom right building)
        self.static_obstacles.append((45.0, 23.0, 6.0, 8.0))

        # Dynamic obstacle 1: Pedestrian crossing vertical crosswalk corridor at x=25.0
        p_cross = DynamicObstacle(x=25.0, y=14.0, vx=0.0, vy=1.0, width=0.8, length=0.8, obs_type="Pedestrian")
        p_cross.base_min_x = 8.0   # Vertical patrol bounds repurposed via vy
        p_cross.base_max_x = 28.0
        self.dynamic_obstacles.append(p_cross)

        # Dynamic obstacle 2: Vehicle cruising along lower traffic lane at y=32.0
        v_traffic = DynamicObstacle(x=20.0, y=32.5, vx=1.8, vy=0.0, width=2.4, length=1.2, obs_type="Vehicle")
        v_traffic.base_min_x = 8.0
        v_traffic.base_max_x = 52.0
        self.dynamic_obstacles.append(v_traffic)

    def update_dynamics(self, dt: float):
        """Updates positions of dynamic moving entities."""
        for dyn in self.dynamic_obstacles:
            dyn.update(dt)

    def get_all_obstacle_boxes(self) -> List[Tuple[float, float, float, float]]:
        """Returns unified list of all obstacle bounding boxes."""
        boxes = list(self.static_obstacles)
        for d in self.dynamic_obstacles:
            boxes.append(d.get_bounding_box())
        return boxes

    def get_obstacle_line_segments(self) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
        """
        Converts all static and dynamic obstacle bounding boxes into
        2D line segments for the LiDAR raycaster.
        """
        segments: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []

        for ox, oy, ow, oh in self.get_all_obstacle_boxes():
            # 4 boundary segments of rectangle
            p1 = (ox, oy)
            p2 = (ox + ow, oy)
            p3 = (ox + ow, oy + oh)
            p4 = (ox, oy + oh)

            segments.append((p1, p2))
            segments.append((p2, p3))
            segments.append((p3, p4))
            segments.append((p4, p1))

        return segments

    def get_dynamic_obstacles_info(self) -> List[Dict[str, Any]]:
        """Returns JSON-serializable representation of dynamic entities for perception."""
        return [
            {
                "x": d.x,
                "y": d.y,
                "vx": d.vx,
                "vy": d.vy,
                "type": d.obs_type,
                "radius": max(d.width, d.length) / 2.0
            }
            for d in self.dynamic_obstacles
        ]

    def check_collision(self, vehicle_corners: Tuple[Tuple[float, float], ...]) -> bool:
        """Checks if any vehicle corner intersects an obstacle box."""
        for vx, vy in vehicle_corners:
            for ox, oy, ow, oh in self.get_all_obstacle_boxes():
                if ox <= vx <= ox + ow and oy <= vy <= oy + oh:
                    return True
        return False
