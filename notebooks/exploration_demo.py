"""
Autonomous Navigation System - Interactive Component Showcase
Demonstrates individual module execution step-by-step for learning & debugging.
"""

import sys
import os
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from config.config import Config
from src.simulation.environment import SimulationEnvironment
from src.planning.a_star import AStarPlanner
from src.perception.lidar import LidarSensor
from src.control.vehicle_model import BicycleVehicleModel

def plot_static_world_and_path():
    """Generates a static Matplotlib plot of the environment and planned A* trajectory."""
    print("Generating Matplotlib plot of environment & planned trajectory...")
    env = SimulationEnvironment()
    planner = AStarPlanner()

    path = planner.plan(env.start_pos, env.goal_pos, env.static_obstacles)

    plt.figure(figsize=(12, 6))
    plt.title("A* Global Planned Route with Inflated Obstacle Barriers")
    plt.xlim(0, Config.WORLD_WIDTH_M)
    plt.ylim(0, Config.WORLD_HEIGHT_M)
    plt.gca().invert_yaxis()

    # Draw static obstacles
    for ox, oy, ow, oh in env.static_obstacles:
        rect = plt.Rectangle((ox, oy), ow, oh, color="#374151", alpha=0.9)
        plt.gca().add_patch(rect)

    # Draw path
    px = [p[0] for p in path]
    py = [p[1] for p in path]
    plt.plot(px, py, color="#10B981", linewidth=3, label="A* Planned Path")
    plt.scatter([env.start_pos[0]], [env.start_pos[1]], color="#3B82F6", s=120, label="Start", zorder=5)
    plt.scatter([env.goal_pos[0]], [env.goal_pos[1]], color="#EF4444", s=120, label="Goal", zorder=5)

    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.4)

    os.makedirs(os.path.join("outputs", "screenshots"), exist_ok=True)
    out_fig = os.path.join("outputs", "screenshots", "a_star_trajectory_plot.png")
    plt.savefig(out_fig, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Plot saved successfully to {out_fig}")

if __name__ == "__main__":
    plot_static_world_and_path()
