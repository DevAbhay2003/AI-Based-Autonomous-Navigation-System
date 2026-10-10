# Technical Architecture Specification

## AI-Based Autonomous Navigation System

This document outlines the software engineering architecture, module interfaces, and data pipelines comprising the autonomous vehicle system.

---

## 1. System Block Diagram

```
+---------------------------------------------------------------------------------+
|                                 WORLD SIMULATION                                |
|  - Urban Roadway Map (Static Walls & Obstacles)                                 |
|  - Dynamic Entities (Pedestrians, Cross-Traffic Vehicles)                       |
+---------------------------------------------------------------------------------+
           |                                                   |
           v                                                   v
+-------------------------------+             +-----------------------------------+
|      2D LIDAR RAYCASTER       |             |     SYNTHETIC CAMERA VISION       |
|  - 360-degree range sweep     |             |  - Roadway Perspective View       |
|  - Laser-obstacle intersection|             |  - Lane Detection & Offset        |
|  - Gaussian measurement noise |             |  - 2D Bounding Box Classifier     |
+-------------------------------+             +-----------------------------------+
           \                                                   /
            \                                                 /
             v                                               v
+---------------------------------------------------------------------------------+
|                              SENSOR FUSION ENGINE                               |
|  - LiDAR Range + Semantic Detections Alignment                                  |
|  - Forward Collision Warning & Dynamic Threat Analysis                          |
+---------------------------------------------------------------------------------+
                                       |
                                       v
+---------------------------------------------------------------------------------+
|                             A* GLOBAL PATH PLANNER                              |
|  - Discrete 8-Connected Metric Grid Map                                         |
|  - Obstacle Costmap Inflation (Safety Buffer Expansion)                         |
|  - Euclidean Admissible Heuristic Search                                        |
|  - Gradient Descent Path Smoothing (Curvature Minimization)                     |
+---------------------------------------------------------------------------------+
                                       |
                                       v
+---------------------------------------------------------------------------------+
|                       LOCAL PLANNER & FINITE STATE MACHINE                      |
|  - Pure Pursuit Path Tracking (Dynamic Lookahead Search)                        |
|  - Reactive Dynamic Obstacle Repulsion (Potential Field Offset)                 |
|  - Finite State Machine: CRUISING | AVOIDING | EMERGENCY_BRAKE | GOAL_REACHED    |
+---------------------------------------------------------------------------------+
                                       |
                     +-----------------+-----------------+
                     |                                   |
                     v                                   v
+----------------------------------+   +------------------------------------------+
|     LONGITUDINAL CONTROLLER      |   |            LATERAL CONTROLLER            |
|  - PID Speed Tracking Controller |   |  - Pure Pursuit Steering Controller      |
|  - Anti-windup Integral Clamping |   |  - Mechanical Steering Angle Limiter     |
+----------------------------------+   +------------------------------------------+
                     \                                   /
                      \                                 /
                       v                               v
+---------------------------------------------------------------------------------+
|                         KINEMATIC BICYCLE MODEL                                 |
|  - Non-holonomic Wheeled Kinematics                                             |
|  - State: [x, y, theta, velocity, steer]                                        |
|  - Forward Euler Numerical Integration (60 Hz)                                  |
+---------------------------------------------------------------------------------+
                                       |
                     +-----------------+-----------------+
                     |                                   |
                     v                                   v
+----------------------------------+   +------------------------------------------+
|      TELEMETRY HUD RENDERER      |   |           PERFORMANCE METRICS            |
|  - Top-down Bird's Eye View Map  |   |  - Trajectory Length & Mission Duration  |
|  - Embedded Forward Camera Feed  |   |  - Min Obstacle Clearance & Collisions   |
|  - Speedometer & Steering Gauges |   |  - Path Smoothness & Jerk Index          |
+----------------------------------+   +------------------------------------------+
```

---

## 2. Module Specifications

### Perception Subsystem (`src/perception/`)
- **LiDAR Sensor (`lidar.py`)**: Raycasting laser scanner simulating 360-degree sweeps with 36-72 rays. Computes distance and intersection coordinates against polygonal obstacles. Features Gaussian sensor noise standard deviation $\sigma = 0.04$ m.
- **Vision Subsystem (`vision.py`)**: Synthesizes a forward-facing camera image (320x200 BGR), renders synthetic lane lines, detects objects within the field-of-view, calculates cross-track lane offsets, and overlays color-coded bounding boxes.
- **Sensor Fusion (`sensor_fusion.py`)**: Aggregates range measurements with vision classifications, categorizing objects into `SAFE`, `WARNING`, and `CRITICAL` hazard zones.

### Planning Subsystem (`src/planning/`)
- **Global Planner (`a_star.py`)**: Computes optimal collision-free paths from arbitrary start to goal coordinates across an inflated obstacle grid. Employs gradient-based trajectory smoothing.
- **Local Planner (`local_planner.py`)**: Operates at 60 Hz to follow global waypoints via Pure Pursuit while repulsing from unexpected dynamic entities.

### Control Subsystem (`src/control/`)
- **Vehicle Model (`vehicle_model.py`)**: Implements non-holonomic bicycle kinematics with wheelbase constraints, acceleration boundaries, and steering angle limits.
- **PID & Pure Pursuit (`pid_controller.py`)**: Computes longitudinal acceleration and steering angle commands.
