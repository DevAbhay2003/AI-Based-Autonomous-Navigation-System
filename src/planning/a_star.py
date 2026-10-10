import math
import heapq
from typing import List, Tuple, Set, Optional
from config.config import Config

class AStarPlanner:
    """
    Industry-standard Global Path Planner implementing A* Search with
    obstacle costmap inflation and geometric path smoothing.
    """

    def __init__(self, world_w: float = Config.WORLD_WIDTH_M,
                 world_h: float = Config.WORLD_HEIGHT_M,
                 grid_res: float = Config.GRID_CELL_SIZE,
                 inflation_r: float = Config.INFLATION_RADIUS):
        self.world_w = world_w
        self.world_h = world_h
        self.res = grid_res
        self.inflation_r = inflation_r
        self.cols = int(world_w / grid_res)
        self.rows = int(world_h / grid_res)

        # 8-connected movement directions: (dx, dy, cost)
        self.motions = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2)),
            (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)),
            (-1, -1, math.sqrt(2))
        ]

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        c = int(x / self.res)
        r = int(y / self.res)
        return (max(0, min(self.cols - 1, c)), max(0, min(self.rows - 1, r)))

    def grid_to_world(self, c: int, r: int) -> Tuple[float, float]:
        return ((c + 0.5) * self.res, (r + 0.5) * self.res)

    def build_costmap(self, obstacles: List[Tuple[float, float, float, float]]) -> Set[Tuple[int, int]]:
        """
        Creates an inflated obstacle grid map.
        Obstacles format: list of (x, y, w, h) bounding boxes in meters.
        """
        occupied: Set[Tuple[int, int]] = set()
        inflation_cells = int(math.ceil(self.inflation_r / self.res))

        for ox, oy, ow, oh in obstacles:
            c_min = max(0, int((ox - self.inflation_r) / self.res))
            c_max = min(self.cols - 1, int((ox + ow + self.inflation_r) / self.res))
            r_min = max(0, int((oy - self.inflation_r) / self.res))
            r_max = min(self.rows - 1, int((oy + oh + self.inflation_r) / self.res))

            for c in range(c_min, c_max + 1):
                for r in range(r_min, r_max + 1):
                    wx, wy = self.grid_to_world(c, r)
                    # Test expanded rectangle
                    if (ox - self.inflation_r <= wx <= ox + ow + self.inflation_r and
                        oy - self.inflation_r <= wy <= oy + oh + self.inflation_r):
                        occupied.add((c, r))

        return occupied

    def plan(self, start: Tuple[float, float], goal: Tuple[float, float],
             obstacles: List[Tuple[float, float, float, float]]) -> List[Tuple[float, float]]:
        """
        Executes A* algorithm from start to goal considering obstacles.
        Returns a smoothed list of waypoints in meters.
        """
        start_cell = self.world_to_grid(*start)
        goal_cell = self.world_to_grid(*goal)
        occupied_cells = self.build_costmap(obstacles)

        if start_cell in occupied_cells:
            # Shift start cell out of obstacle if accidentally inside margin
            occupied_cells.discard(start_cell)
        if goal_cell in occupied_cells:
            occupied_cells.discard(goal_cell)

        # Priority queue for open set: (f_score, c, r)
        open_set = []
        heapq.heappush(open_set, (0.0, start_cell[0], start_cell[1]))

        came_from = {}
        g_score = {start_cell: 0.0}

        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            # Euclidean distance heuristic
            return math.hypot(a[0] - b[0], a[1] - b[1])

        found = False

        while open_set:
            _, curr_c, curr_r = heapq.heappop(open_set)
            current = (curr_c, curr_r)

            if current == goal_cell:
                found = True
                break

            for dx, dy, move_cost in self.motions:
                neighbor = (curr_c + dx, curr_r + dy)

                if not (0 <= neighbor[0] < self.cols and 0 <= neighbor[1] < self.rows):
                    continue
                if neighbor in occupied_cells:
                    continue

                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal_cell)
                    heapq.heappush(open_set, (f_score, neighbor[0], neighbor[1]))

        if not found:
            # Fallback direct line if completely blocked
            return [start, goal]

        # Reconstruct path from goal back to start
        path_cells = []
        curr = goal_cell
        while curr in came_from:
            path_cells.append(curr)
            curr = came_from[curr]
        path_cells.append(start_cell)
        path_cells.reverse()

        # Convert grid cells to real-world coordinates
        raw_path = [self.grid_to_world(c, r) for c, r in path_cells]
        raw_path[0] = start
        raw_path[-1] = goal

        # Smooth the discrete jagged path
        return self._smooth_path(raw_path)

    def _smooth_path(self, path: List[Tuple[float, float]], weight_data: float = 0.5,
                     weight_smooth: float = 0.35, tolerance: float = 0.001) -> List[Tuple[float, float]]:
        """Gradient descent path smoothing for curvature reduction."""
        if len(path) <= 2:
            return path

        new_path = [list(pt) for pt in path]
        change = tolerance
        max_iter = 100
        iteration = 0

        while change >= tolerance and iteration < max_iter:
            change = 0.0
            iteration += 1
            for i in range(1, len(path) - 1):
                for d in range(2):
                    orig = new_path[i][d]
                    new_val = orig + weight_data * (path[i][d] - orig) + \
                              weight_smooth * (new_path[i-1][d] + new_path[i+1][d] - (2.0 * orig))
                    change += abs(orig - new_val)
                    new_path[i][d] = new_val

        return [(pt[0], pt[1]) for pt in new_path]
