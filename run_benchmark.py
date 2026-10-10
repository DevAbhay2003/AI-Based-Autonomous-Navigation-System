import os
import sys
import time
import json
import math
from typing import Dict, Any, List

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from src.utils.logger import setup_logger
from src.utils.metrics import PerformanceMetrics
from src.control.vehicle_model import BicycleVehicleModel
from src.perception.lidar import LidarSensor
from src.planning.a_star import AStarPlanner
from src.planning.local_planner import LocalPlanner, NavigationState
from src.simulation.environment import SimulationEnvironment, DynamicObstacle

logger = setup_logger("AutoNavBenchmark")

def run_headless_scenario(scenario_id: int, name: str, start: tuple, goal: tuple,
                          dynamic_density: int, max_steps: int = 1200) -> Dict[str, Any]:
    """Runs an individual navigation scenario headlessly without GUI."""
    logger.info(f"--- Running Benchmark Scenario {scenario_id}: '{name}' ---")
    env = SimulationEnvironment()
    env.start_pos = start
    env.goal_pos = goal
    env.setup_default_map()

    # Adjust dynamic density
    if dynamic_density == 0:
        env.dynamic_obstacles.clear()
    elif dynamic_density >= 3:
        env.dynamic_obstacles.append(
            DynamicObstacle(x=30.0, y=10.0, vx=1.0, vy=0.0, obs_type="Vehicle")
        )

    vehicle = BicycleVehicleModel(wheelbase=Config.WHEELBASE, max_steer=Config.MAX_STEER_ANGLE)
    vehicle.set_pose(x=start[0], y=start[1], yaw=0.0, v=0.0)

    planner = AStarPlanner()
    local_planner = LocalPlanner()
    lidar = LidarSensor()
    metrics = PerformanceMetrics()

    t0 = time.time()
    global_path = planner.plan(start=start, goal=goal, obstacles=env.static_obstacles)
    planning_time = (time.time() - t0) * 1000.0

    dt = 1.0 / 60.0
    success = False

    for step in range(max_steps):
        env.update_dynamics(dt)
        dynamic_info = env.get_dynamic_obstacles_info()
        segments = env.get_obstacle_line_segments()

        lidar_data = lidar.scan(vehicle.state.x, vehicle.state.y, vehicle.state.yaw, segments)

        throttle, steer, state = local_planner.update(
            vehicle.state.x, vehicle.state.y, vehicle.state.yaw, vehicle.state.v,
            global_path, lidar_data.forward_min_distance, dynamic_info, dt
        )

        vehicle.update(throttle, steer, dt)

        corners = vehicle.get_bounding_box(Config.VEHICLE_LENGTH, Config.VEHICLE_WIDTH)
        if env.check_collision(corners):
            metrics.register_collision()

        metrics.update(vehicle.state.x, vehicle.state.y, vehicle.state.v, vehicle.state.steer,
                       lidar_data.min_distance, dt)

        if state == NavigationState.GOAL_REACHED:
            success = True
            metrics.goal_reached = True
            break

    summary = metrics.summary()
    summary["scenario_id"] = scenario_id
    summary["scenario_name"] = name
    summary["planning_latency_ms"] = round(planning_time, 2)
    return summary

def run_suite():
    """Executes test suite of varied navigation scenarios."""
    scenarios = [
        (1, "Baseline Clear Highway", (5.0, 18.0), (55.0, 18.0), 0),
        (2, "Urban Obstacle Chicane", (5.0, 8.0), (52.0, 26.0), 1),
        (3, "High-Density Dynamic Traffic", (6.0, 20.0), (54.0, 15.0), 3),
    ]

    results = []
    print("\n" + "=" * 65)
    print("      EXECUTING HEADLESS AUTONOMOUS NAVIGATION BENCHMARKS")
    print("=" * 65)

    for sid, name, start, goal, dyn_count in scenarios:
        res = run_headless_scenario(sid, name, start, goal, dyn_count)
        results.append(res)
        status_str = "PASS" if res["mission_success"] and res["total_collisions"] == 0 else "FAIL"
        print(f"Scenario {sid} [{name}]: {status_str} | Time: {res['total_time_sec']}s | Collisions: {res['total_collisions']} | Dist: {res['distance_traveled_m']}m")

    out_file = os.path.join("outputs", "benchmark_suite_results.json")
    os.makedirs("outputs", exist_ok=True)
    with open(out_file, "w") as f:
        json.dump(results, f, indent=4)

    print("\nBenchmark suite completed successfully. Results saved to:", out_file)
    print("=" * 65 + "\n")

if __name__ == "__main__":
    run_suite()
