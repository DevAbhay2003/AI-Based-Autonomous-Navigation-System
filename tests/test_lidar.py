import unittest
from src.perception.lidar import LidarSensor

class TestLidarSensor(unittest.TestCase):
    def setUp(self):
        self.lidar = LidarSensor(num_rays=16, max_range=10.0, noise_std=0.0)

    def test_wall_detection_distance(self):
        # Ray straight forward toward a wall at x = 5.0
        origin_x, origin_y = 0.0, 0.0
        yaw = 0.0 # Facing +X
        # Wall segment from (5.0, -5.0) to (5.0, 5.0)
        segments = [((5.0, -5.0), (5.0, 5.0))]

        res = self.lidar.scan(origin_x, origin_y, yaw, segments)
        # Should detect wall at approximately 5.0 meters
        self.assertAlmostEqual(res.min_distance, 5.0, delta=0.5)

if __name__ == "__main__":
    unittest.main()
