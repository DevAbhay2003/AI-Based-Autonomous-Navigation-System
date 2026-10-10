import unittest
from src.planning.a_star import AStarPlanner

class TestAStarPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = AStarPlanner(world_w=30.0, world_h=30.0, grid_res=1.0, inflation_r=1.0)

    def test_straight_line_planning(self):
        start = (2.0, 15.0)
        goal = (25.0, 15.0)
        obstacles = []
        path = self.planner.plan(start, goal, obstacles)

        self.assertGreater(len(path), 1)
        self.assertAlmostEqual(path[0][0], start[0], places=1)
        self.assertAlmostEqual(path[0][1], start[1], places=1)
        self.assertAlmostEqual(path[-1][0], goal[0], places=1)
        self.assertAlmostEqual(path[-1][1], goal[1], places=1)

    def test_obstacle_avoidance_path(self):
        start = (5.0, 15.0)
        goal = (25.0, 15.0)
        # Block direct path with obstacle in center
        obstacles = [(14.0, 10.0, 4.0, 10.0)]
        path = self.planner.plan(start, goal, obstacles)

        # Path must detour around the obstacle
        detoured = any(y > 20.0 or y < 10.0 for _, y in path)
        self.assertTrue(detoured, "Path should detour around blocking obstacle")

if __name__ == "__main__":
    unittest.main()
