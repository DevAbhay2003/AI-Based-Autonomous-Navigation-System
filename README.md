# 🚗 AI-Based Autonomous Navigation System

[![Python Version](https://img.shields.io/badge/Python-3.9%20%7C%203.10%20%7C%203.11%20%7C%203.12%20%7C%203.13-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Autonomous Driving](https://img.shields.io/badge/Robotics-Autonomous%20Navigation-brightgreen.svg)](#)
[![Simulation](https://img.shields.io/badge/Simulation-60%20FPS%20Real--Time-orange.svg)](#)
[![Tests Passing](https://img.shields.io/badge/Tests-5%2F5%20Passing-brightgreen.svg)](#)

> An industry-oriented, end-to-end autonomous vehicle navigation stack featuring synthetic multi-sensor perception (2D LiDAR + Frontal Computer Vision Camera), sensor fusion, global $A^*$ grid path planning with obstacle inflation, reactive local dynamic obstacle evasion, kinematic bicycle model physics, and a real-time telemetry HUD dashboard.

---

## 📌 Table of Contents
1. [Project Overview](#-project-overview)
2. [Problem Statement & Industry Relevance](#-problem-statement--industry-relevance)
3. [System Architecture & Dataflow](#-system-architecture--dataflow)
4. [Folder Structure](#-folder-structure)
5. [Prerequisites & Installation](#-prerequisites--installation)
6. [Quick Start: Running the Project](#-quick-start-running-the-project)
7. [Simulation Workflow & Telemetry HUD](#-simulation-workflow--telemetry-hud)
8. [Automated Headless Benchmarks & CI/CD Testing](#-automated-headless-benchmarks--cicd-testing)
9. [Quantitative Performance Results](#-quantitative-performance-results)
10. [Proof of Work & Output Artifacts](#-proof-of-work--output-artifacts)
11. [Future Roadmap](#-future-roadmap)
12. [Author & Portfolio Contact](#-author--portfolio-contact)

---

## 🌟 Project Overview

Autonomous systems operate across a closed-loop triad: **Perception $\rightarrow$ Planning $\rightarrow$ Control**. This repository implements a fully self-contained, hardware-free autonomous navigation platform designed to run virtually on standard consumer hardware while maintaining strict fidelity to modern automotive and robotics engineering paradigms (e.g., Waymo, Cruise, Tesla FSD, and Amazon Robotics AGVs).

### Core Capabilities
- **Synthetic Multi-Modal Perception**:
  - **2D LiDAR Scanner**: 360° radial raycasting with configurable beam density, ray-box intersection testing, and Gaussian noise simulation ($\sigma = 0.04\text{ m}$).
  - **Front-Facing Vision Camera**: Synthesizes a forward driving perspective, performs simulated lane boundary detection, predicts lateral cross-track offsets, and computes 2D bounding boxes with classification labels (`Vehicle`, `Pedestrian`, `Obstacle`).
- **Sensor Fusion Engine**: Integrates distance metrics from LiDAR and bounding box classifications into an obstacle threat matrix, computing Time-To-Collision (TTC) and dynamic risk levels (`NORMAL`, `CAUTION`, `EMERGENCY_STOP`).
- **Two-Tier Hierarchical Motion Planning**:
  - **Global Planner**: Discrete grid-based $A^*$ pathfinding augmented with obstacle costmap inflation (safety margin $R_{inf} = 2.0\text{ m}$) and gradient descent curvature smoothing.
  - **Local Planner & FSM**: Real-time Pure Pursuit path tracking combined with reactive repulsive potential vectors for avoiding moving pedestrians and vehicles.
- **Realistic Kinematic Vehicle Model**: Implements the Non-Holonomic Kinematic Bicycle Model with steering limits ($\delta_{\max} \approx 37^\circ$), acceleration, and braking dynamics.
- **Glass-Cockpit Telemetry HUD**: Interactive 60 FPS dashboard presenting Bird's Eye View (BEV) map overlays, live camera feed insets, digital speedometers, steering gauges, LiDAR hit clouds, and active FSM states.

---

## 🏭 Problem Statement & Industry Relevance

### The Challenge
Developing software for physical robots and autonomous vehicles requires extensive hardware, safety test tracks, and high capital expense. Engineers and students need accessible, robust, and rigorous virtual testbeds that mirror production robotics stacks to rapidly prototype and evaluate perception, planning, and control algorithms.

### Industrial Applications
- **Self-Driving Vehicles (L4/L5 Autonomous Mobility)**: Urban path navigation, pedestrian collision mitigation, lane keeping, and dynamic vehicle passing.
- **Automated Guided Vehicles (AGV) & AMRs**: Warehouse fulfillment robots navigating dynamic fulfillment centers (e.g., Amazon Robotics, KIVA, Ocado).
- **Last-Mile Delivery Robots**: Sidewalk and suburban parcel delivery units navigating sidewalks with pedestrians.
- **Defense & Industrial Inspection**: Unmanned Ground Vehicles (UGVs) exploring unstructured or hazardous terrain.

---

## 🏗️ System Architecture & Dataflow

```
[Simulation World & Dynamic Actors]
                 |
                 +-----------------------+
                 |                       |
                 v                       v
      [360° LiDAR Raycaster]   [Frontal Vision Subsystem]
                 |                       |
                 +-----------+-----------+
                             |
                             v
                  [Sensor Fusion Engine]
                             |
                             v
                 [A* Global Grid Planner]
                 (Inflated Costmap + Smoothing)
                             |
                             v
           [Reactive Local Planner & Safety FSM]
           (Pure Pursuit + Potential Field Repulsion)
                             |
                 +-----------+-----------+
                 |                       |
                 v                       v
        [Longitudinal PID]       [Lateral Pure Pursuit]
                 \                       /
                  \                     /
                   v                   v
              [Kinematic Bicycle Model Physics]
                             |
                             v
          [Real-Time Visualizer HUD & Telemetry Logs]
```

---

## 📂 Folder Structure

```
AI-Autonomous-Navigation-System/
├── config/
│   ├── __init__.py
│   └── config.py               # Hyperparameters (dimensions, physics limits, planner thresholds)
├── docs/
│   ├── architecture.md         # In-depth architectural blueprint and block diagrams
│   ├── math_foundations.md     # Kinematic bicycle equations, A* admissibility, Pure Pursuit math
│   └── api_reference.md        # API documentation for public classes and methods
├── outputs/
│   ├── screenshots/            # Automated telemetry and mission proof captures
│   ├── benchmark_results.json  # Quantitative metrics from visual simulation runs
│   └── benchmark_suite_results.json # Quantitative results across 3 test scenarios
├── src/
│   ├── __init__.py
│   ├── control/
│   │   ├── __init__.py
│   │   ├── vehicle_model.py    # Non-holonomic bicycle kinematics simulation
│   │   └── pid_controller.py   # Longitudinal PID & Lateral Pure Pursuit controller
│   ├── perception/
│   │   ├── __init__.py
│   │   ├── lidar.py            # 2D LiDAR raycaster with sensor noise
│   │   ├── vision.py           # Front-camera perspective view, lane tracking, object detection
│   │   └── sensor_fusion.py    # Sensor fusion engine and collision threat estimator
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── a_star.py           # A* global path planning with costmap inflation & smoothing
│   │   └── local_planner.py    # Reactive local planner, potential field repulsion, FSM
│   ├── simulation/
│   │   ├── __init__.py
│   │   ├── environment.py      # Map geometry, static buildings, moving pedestrians/vehicles
│   │   └── renderer.py         # Pygame & OpenCV telemetry visualizer HUD
│   └── utils/
│       ├── __init__.py
│       ├── logger.py           # Formatted system logger
│       └── metrics.py          # Benchmark tracker (path length, clearance, jerk, collisions)
├── tests/
│   ├── __init__.py
│   ├── test_a_star.py          # Verification of A* path optimality & avoidance
│   ├── test_lidar.py           # Verification of LiDAR ray range calculation
│   └── test_vehicle.py         # Verification of bicycle kinematic state equations
├── .gitignore                  # Git repository exclusion rules
├── main.py                     # Primary executable entrypoint (Interactive 60 FPS Visual Simulation)
├── requirements.txt            # Python package dependencies
└── run_benchmark.py            # Automated headless scenario test suite (CI/CD ready)
```

---

## ⚙️ Prerequisites & Installation

### 1. Requirements
- Python 3.9+ (tested on Python 3.10, 3.11, 3.12, 3.13)
- Windows 10/11, macOS, or Linux (Ubuntu 20.04/22.04+)
- No dedicated GPU required! Runs at 60 FPS on standard laptop CPUs.

### 2. Setup Virtual Environment

#### On Windows (PowerShell):
```powershell
# Clone the repository
git clone https://github.com/your-username/AI-Autonomous-Navigation-System.git
cd AI-Autonomous-Navigation-System

# Create virtual environment
python -m venv venv

# Activate virtual environment
.\venv\Scripts\Activate.ps1

# Upgrade pip & install dependencies
python -m pip install --upgrade pip
pip install -r requirements.txt
```

#### On Linux / macOS:
```bash
# Clone the repository
git clone https://github.com/your-username/AI-Autonomous-Navigation-System.git
cd AI-Autonomous-Navigation-System

# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Upgrade pip & install dependencies
pip install --upgrade pip
pip install -r requirements.txt
```

### 3. Verify Installation
Run the unit test suite:
```bash
python -m unittest discover tests
```
Expected output:
```text
.....
----------------------------------------------------------------------
Ran 5 tests in 0.003s

OK
```

---

## 🚀 Quick Start: Running the Project

### 1. Launch Interactive Visual Simulation
Launch the real-time simulation with active HUD and computer vision insets:
```bash
python main.py
```

#### Keyboard & Mouse Interactive Controls
| Control Key | Action |
| :--- | :--- |
| `L` | Toggle 360° LiDAR laser beams on/off |
| `P` | Toggle planned global path line overlay |
| `S` | Instantly capture a high-resolution screenshot to `outputs/screenshots/` |
| `ESC` / Close | Exit simulation cleanly and export benchmark report |

---

## 🖥️ Simulation Workflow & Telemetry HUD

The simulation visualizer provides a unified split-cockpit display:
1. **Bird's Eye View (BEV) World**:
   - Asphalt roadways with lane divisions.
   - Solid urban obstacles (buildings, concrete dividers).
   - Moving dynamic entities (pedestrians crossing, oncoming traffic).
   - Ego vehicle chassis with live wheel steering orientation.
   - Emerald green global $A^*$ route with waypoint anchors.
   - Indigo path trajectory trail showing historic driving behavior.
   - Red laser impact cloud tracking detected collision frontiers.
2. **Embedded Front-Facing Camera Inset (Top Right)**:
   - Synthetic camera image with horizon and perspective roadway geometry.
   - Real-time lane boundary detection and cross-track offset readout.
   - Vision-detected obstacle bounding boxes with category tags (`Pedestrian`, `Vehicle`) and estimated distance.
3. **Telemetry Instrument Panel (Top Left)**:
   - Real-time vehicle speed (km/h and m/s).
   - Wheel steering deflection angle.
   - Distance remaining to target beacon.
   - Minimum LiDAR obstacle clearance margin.
   - Active Finite State Machine state (`CRUISING`, `AVOIDING_OBSTACLE`, `EMERGENCY_BRAKE`, `GOAL_REACHED`).

---

## 🧪 Automated Headless Benchmarks & CI/CD Testing

To test algorithms without opening a window (e.g., in continuous integration workflows or batch scenario evaluations), run:
```bash
python run_benchmark.py
```

### Benchmark Scenarios Evaluated:
1. **Scenario 1: Baseline Clear Highway** — Long-range unhindered cruising, testing trajectory stability and PID velocity settling time.
2. **Scenario 2: Urban Obstacle Chicane** — Narrow corridor navigation around static building blocks, testing A* path inflation and Pure Pursuit cornering.
3. **Scenario 3: High-Density Dynamic Traffic** — Active avoidance of crossing pedestrians and moving vehicles, testing sensor fusion and reactive emergency braking (AEB).

---

## 📊 Quantitative Performance Results

Results generated across automated benchmark scenarios:

| Metric | Scenario 1 (Highway) | Scenario 2 (Chicane) | Scenario 3 (Dynamic Traffic) |
| :--- | :---: | :---: | :---: |
| **Mission Outcome** | **PASS (100%)** | **PASS (100%)** | **PASS (100%)** |
| **Collisions** | **0** | **0** | **0** |
| **Mission Duration** | 16.03 s | 17.35 s | 18.92 s |
| **Distance Traveled** | 51.70 m | 55.81 m | 50.64 m |
| **Average Speed** | 3.22 m/s (11.6 km/h) | 3.21 m/s (11.5 km/h) | 2.67 m/s (9.6 km/h) |
| **Min Clearance** | 3.82 m | 2.05 m | 2.24 m |
| **Planning Latency** | 3.42 ms | 5.81 ms | 3.88 ms |

Benchmark JSON reports are automatically exported to `outputs/benchmark_results.json`.

---

## 📸 Proof of Work & Output Artifacts

All runs automatically generate visual proof saved in `outputs/screenshots/`:
- `outputs/screenshots/autonomous_navigation_simulation.png` — Real-time telemetry dashboard during navigation.
- `outputs/screenshots/mission_goal_reached.png` — Final goal destination arrival confirmation.

---

## 🔮 Future Roadmap

- [ ] **CARLA Simulator Bridge**: Export waypoint trajectories to the photorealistic CARLA Unreal Engine simulator via Python API.
- [ ] **ROS 2 Integration**: Wrap modules into standard ROS 2 nodes (`sensor_msgs/LaserScan`, `nav_msgs/Odometry`, `geometry_msgs/Twist`).
- [ ] **Deep Reinforcement Learning (PPO/SAC)**: Train end-to-end continuous steering policies in Gym-compatible wrappers.
- [ ] **LiDAR SLAM (Fast-SLAM / Cartographer)**: Real-time map generation in unknown environments without prior global maps.

---

## 👤 Author & Portfolio Contact

- **Project Lead**: **Abhishek Basu**
- **GitHub**: [DevAbhay2003](https://github.com/DevAbhay2003?tab=repositories)
- **LinkedIn**: [https://linkedin.com/in/your-profile](https://www.linkedin.com/in/abhishek-basu-68b1b1342/)
- **Email**: devabhishekbasu03@gmail.com

---
*Built with ❤️ for robotics and autonomous driving portfolio excellence.*
