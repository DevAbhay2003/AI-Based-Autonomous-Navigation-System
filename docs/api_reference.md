# API Reference & Module Documentation

This document describes key classes, methods, and configurations available in the **AI-Based Autonomous Navigation System**.

---

## 1. `config.config.Config`
Centralized repository containing all hyper-parameters for physics, simulation geometry, sensor specs, and planner algorithms.

| Parameter | Default | Units | Description |
| :--- | :--- | :--- | :--- |
| `SCREEN_WIDTH` | 1280 | pixels | Pygame screen display width |
| `SCREEN_HEIGHT` | 720 | pixels | Pygame screen display height |
| `SIMULATION_SCALE` | 20.0 | px/m | Coordinate mapping: 20 pixels per 1 meter |
| `WORLD_WIDTH_M` | 64.0 | meters | Planar world width |
| `WORLD_HEIGHT_M` | 36.0 | meters | Planar world height |
| `WHEELBASE` | 1.8 | meters | Distance between vehicle front and rear axles |
| `MAX_STEER_ANGLE` | 0.65 | rad | Maximum front wheel steering deflection (~37°) |
| `MAX_SPEED` | 6.0 | m/s | Top cruising velocity (21.6 km/h) |
| `LIDAR_NUM_RAYS` | 36 | beams | Ray count for 360-degree sweep |
| `LIDAR_MAX_RANGE` | 12.0 | meters | Maximum laser measurement distance |
| `INFLATION_RADIUS` | 2.0 | meters | Obstacle safety inflation radius for A* search |

---

## 2. Perception Subsystem

### `src.perception.lidar.LidarSensor`
```python
class LidarSensor:
    def __init__(self, num_rays: int, max_range: float, noise_std: float)
    def scan(self, origin_x: float, origin_y: float, orientation_yaw: float,
             obstacle_segments: List[Tuple[Tuple[float, float], Tuple[float, float]]]) -> LidarScanResult
```
- Performs radial raycasting against 2D line segments.
- Computes `min_distance` (360°) and `forward_min_distance` (±45° forward cone).
- Synthesizes Gaussian measurement noise to simulate real-world sensor drift.

### `src.perception.vision.VisionSubsystem`
```python
class VisionSubsystem:
    def __init__(self, fov_deg: float = 90.0, max_range: float = 15.0)
    def generate_camera_frame(self, ego_x: float, ego_y: float, ego_yaw: float,
                              obstacles: List[Dict[str, Any]]) -> Tuple[np.ndarray, List[DetectedObject], float]
```
- Renders synthetic perspective road view with lane lines.
- Detects dynamic obstacle bounding boxes within camera FOV.
- Outputs estimated cross-track lane departure offset.

---

## 3. Planning Subsystem

### `src.planning.a_star.AStarPlanner`
```python
class AStarPlanner:
    def plan(self, start: Tuple[float, float], goal: Tuple[float, float],
             obstacles: List[Tuple[float, float, float, float]]) -> List[Tuple[float, float]]
```
- Converts continuous world bounds into inflated grid cells.
- Evaluates cost using an admissible Euclidean heuristic.
- Applies iterative gradient descent smoothing on extracted waypoints.

### `src.planning.local_planner.LocalPlanner`
```python
class LocalPlanner:
    def update(self, current_x: float, current_y: float, current_yaw: float, current_v: float,
               global_path: List[Tuple[float, float]], min_lidar_dist: float,
               dynamic_obstacles: List[Dict[str, Any]], dt: float) -> Tuple[float, float, NavigationState]
```
- Tracks planned waypoints using Pure Pursuit.
- Applies dynamic obstacle repulsion vectors.
- Manages Finite State Machine transitions (`CRUISING`, `AVOIDING_OBSTACLE`, `EMERGENCY_BRAKE`, `GOAL_REACHED`).

---

## 4. Control Subsystem

### `src.control.vehicle_model.BicycleVehicleModel`
```python
class BicycleVehicleModel:
    def update(self, throttle_accel: float, target_steer: float, dt: float)
    def get_bounding_box(self, length: float, width: float) -> Tuple[Tuple[float, float], ...]
```
- Simulates non-holonomic kinematics via forward Euler integration.
- Calculates exact 4-corner oriented bounding box for collision detection.
