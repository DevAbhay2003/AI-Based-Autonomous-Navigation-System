import unittest
import math
from src.control.vehicle_model import BicycleVehicleModel

class TestVehicleModel(unittest.TestCase):
    def setUp(self):
        self.vehicle = BicycleVehicleModel(wheelbase=2.0, max_steer=0.5, max_speed=10.0)

    def test_straight_line_acceleration(self):
        self.vehicle.set_pose(0.0, 0.0, 0.0, 0.0)
        dt = 0.1
        # Accelerate at 2 m/s^2 for 1 second (10 steps)
        for _ in range(10):
            self.vehicle.update(throttle_accel=2.0, target_steer=0.0, dt=dt)

        # Final speed should be approx 2.0 m/s
        self.assertAlmostEqual(self.vehicle.state.v, 2.0, delta=0.1)
        # Position x should have advanced forward
        self.assertGreater(self.vehicle.state.x, 0.5)
        # Heading yaw should remain approx 0
        self.assertAlmostEqual(self.vehicle.state.yaw, 0.0, delta=0.05)

    def test_turning_behavior(self):
        self.vehicle.set_pose(0.0, 0.0, 0.0, 4.0)
        dt = 0.1
        # Steer to the left (positive steer)
        for _ in range(10):
            self.vehicle.update(throttle_accel=0.0, target_steer=0.3, dt=dt)

        # Vehicle yaw should have increased
        self.assertGreater(self.vehicle.state.yaw, 0.0)

if __name__ == "__main__":
    unittest.main()
