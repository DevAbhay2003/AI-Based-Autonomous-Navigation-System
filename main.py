import os
import sys
import time
import math
from typing import List, Tuple

# Add workspace directory to python path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config.config import Config
from src.utils.logger import setup_logger
from src.utils.metrics import PerformanceMetrics
from src.control.vehicle_model import BicycleVehicleModel
from src.perception.lidar import LidarSensor
from src.perception.vision import VisionSubsystem
from src.perception.sensor_fusion import SensorFusionEngine
from src.planning.a_star import AStarPlanner
from src.planning.local_planner import LocalPlanner, NavigationState
from src.simulation.environment import SimulationEnvironment
from src.simulation.renderer import SimulationRenderer

logger = setup_logger("AutoNavMain")

def run_simulation(auto_screenshot: bool = True, max_duration_sec: float = 45.0):
    """
    Main Autonomous Navigation Execution Loop.
    Integrates Perception, Fusion, Planning, Control, and Telemetry Visualization.
    """
    logger.info("Initializing AI-Based Autonomous Navigation System...")

    # 1. Initialize Subsystems
    env = SimulationEnvironment()
    vehicle = BicycleVehicleModel(
        wheelbase=Config.WHEELBASE,
        max_steer=Config.MAX_STEER_ANGLE,
        max_speed=Config.MAX_SPEED
    )
    vehicle.set_pose(x=env.start_pos[0], y=env.start_pos[1], yaw=0.0, v=0.0)

    lidar = LidarSensor()
    vision = VisionSubsystem()
    fusion = SensorFusionEngine()
    global_planner = AStarPlanner()
    local_planner = LocalPlanner()
    metrics = PerformanceMetrics()
    renderer = SimulationRenderer()

    # 2. Compute Global Path via A*
    logger.info(f"Computing A* Global Path: Start {env.start_pos} -> Goal {env.goal_pos}...")
    t0_plan = time.time()
    global_path = global_planner.plan(
        start=env.start_pos,
        goal=env.goal_pos,
        obstacles=env.static_obstacles
    )
    plan_time_ms = (time.time() - t0_plan) * 1000.0
    logger.info(f"Global Path Generated: {len(global_path)} waypoints in {plan_time_ms:.2f} ms")

    # Tracking trail and timing
    history_trail: List[Tuple[float, float]] = []
    dt = 1.0 / Config.FPS
    running = True
    screenshot_taken = False
    start_sim_time = time.time()

    logger.info("Entering Real-Time Simulation Loop (60 FPS)...")

    while running:
        # Check simulation timeout
        elapsed_sec = time.time() - start_sim_time
        if elapsed_sec > max_duration_sec:
            logger.info("Maximum simulation duration reached. Ending run.")
            break

        # A. Update Environment Dynamic Entities
        env.update_dynamics(dt)
        dynamic_obs_info = env.get_dynamic_obstacles_info()
        obstacle_segments = env.get_obstacle_line_segments()

        # B. Perception Subsystem: LiDAR Scan
        lidar_result = lidar.scan(
            origin_x=vehicle.state.x,
            origin_y=vehicle.state.y,
            orientation_yaw=vehicle.state.yaw,
            obstacle_segments=obstacle_segments
        )

        # C. Perception Subsystem: Computer Vision Camera Frame
        camera_frame, detected_objs, lane_offset = vision.generate_camera_frame(
            ego_x=vehicle.state.x,
            ego_y=vehicle.state.y,
            ego_yaw=vehicle.state.yaw,
            obstacles=dynamic_obs_info
        )

        # D. Sensor Fusion: Range + Vision Threat Evaluation
        fused_objects, threat_status, min_clearance = fusion.fuse(
            lidar_data=lidar_result,
            vision_objects=detected_objs,
            current_speed=vehicle.state.v
        )

        # E. Planning Subsystem: Reactive Local Planning
        throttle_accel, steer_angle, nav_state = local_planner.update(
            current_x=vehicle.state.x,
            current_y=vehicle.state.y,
            current_yaw=vehicle.state.yaw,
            current_v=vehicle.state.v,
            global_path=global_path,
            min_lidar_dist=lidar_result.forward_min_distance,
            dynamic_obstacles=dynamic_obs_info,
            dt=dt
        )

        # F. Control Subsystem: Kinematic Bicycle Model Update
        vehicle.update(throttle_accel=throttle_accel, target_steer=steer_angle, dt=dt)

        # G. Safety & Collision Monitoring
        corners = vehicle.get_bounding_box(Config.VEHICLE_LENGTH, Config.VEHICLE_WIDTH)
        if env.check_collision(corners):
            metrics.register_collision()
            logger.warning("Collision Alert! Vehicle bounding box intersected obstacle boundary.")

        # H. Metrics Recording
        metrics.update(
            x=vehicle.state.x,
            y=vehicle.state.y,
            velocity=vehicle.state.v,
            steer=vehicle.state.steer,
            min_clearance=min_clearance,
            dt=dt
        )
        history_trail.append((vehicle.state.x, vehicle.state.y))

        # I. Rendering Visualizer HUD
        active = renderer.render_frame(
            vehicle=vehicle,
            static_obstacles=env.static_obstacles,
            dynamic_obstacles=env.dynamic_obstacles,
            goal_pos=env.goal_pos,
            global_path=global_path,
            history_trail=history_trail,
            lidar_data=lidar_result,
            camera_bgr_frame=camera_frame,
            nav_state=nav_state,
            min_clearance=min_clearance
        )
        if not active:
            logger.info("Window closed by user.")
            break

        # Capture mid-run demonstration screenshot for GitHub portfolio
        if auto_screenshot and not screenshot_taken and elapsed_sec > 4.0:
            shot_path = os.path.join("outputs", "screenshots", "autonomous_navigation_simulation.png")
            renderer.save_screenshot(shot_path)
            logger.info(f"Captured portfolio proof screenshot to {shot_path}")
            screenshot_taken = True

        # Check Goal Condition
        if nav_state == NavigationState.GOAL_REACHED:
            metrics.goal_reached = True
            logger.info("MISSION SUCCESS! Autonomous vehicle successfully reached destination goal.")
            # Capture final goal proof
            shot_path = os.path.join("outputs", "screenshots", "mission_goal_reached.png")
            renderer.save_screenshot(shot_path)
            time.sleep(1.0)
            break

    # Save performance evaluation report
    report_path = os.path.join("outputs", "benchmark_results.json")
    metrics.save_to_file(report_path)
    logger.info(f"Performance benchmark saved to {report_path}")

    # Display Mission Report Summary in Console
    summary = metrics.summary()
    print("\n" + "=" * 55)
    print("      AUTONOMOUS NAVIGATION MISSION REPORT")
    print("=" * 55)
    print(f" Mission Status:             {'SUCCESS (GOAL REACHED)' if summary['mission_success'] else 'COMPLETED'}")
    print(f" Total Mission Time:         {summary['total_time_sec']} seconds")
    print(f" Total Distance Traveled:    {summary['distance_traveled_m']} meters")
    print(f" Average Velocity:           {summary['average_speed_kmh']} km/h ({summary['average_speed_mps']} m/s)")
    print(f" Minimum Safety Clearance:   {summary['min_obstacle_clearance_m']} meters")
    print(f" Collisions Registered:      {summary['total_collisions']}")
    print(f" Steering Smoothness Index:  {summary['steering_smoothness_index']}")
    print("=" * 55 + "\n")

    renderer.close()

if __name__ == "__main__":
    run_simulation()
