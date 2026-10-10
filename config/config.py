import os

class Config:
    """Central configuration for AI Autonomous Navigation System."""

    # Screen & Simulation Settings
    SCREEN_WIDTH = 1280
    SCREEN_HEIGHT = 720
    FPS = 60
    SIMULATION_SCALE = 20.0  # Pixels per meter (1m = 20px)

    # World Bounds (in meters)
    WORLD_WIDTH_M = SCREEN_WIDTH / SIMULATION_SCALE   # 64 meters
    WORLD_HEIGHT_M = SCREEN_HEIGHT / SIMULATION_SCALE # 36 meters

    # Grid Map Resolution for Global Path Planning
    GRID_CELL_SIZE = 1.0  # 1 meter per cell
    INFLATION_RADIUS = 2.0  # Safety margin radius around obstacles (meters)

    # Vehicle Kinematics (Bicycle Model)
    VEHICLE_LENGTH = 2.4   # meters
    VEHICLE_WIDTH = 1.2    # meters
    WHEELBASE = 1.8        # meters
    MAX_STEER_ANGLE = 0.65 # radians (~37 degrees)
    MAX_SPEED = 6.0        # m/s (~21.6 km/h)
    MIN_SPEED = 0.0        # m/s
    MAX_ACCELERATION = 2.5 # m/s^2
    MAX_BRAKING = 4.0      # m/s^2

    # Pure Pursuit Lateral Controller
    LOOKAHEAD_DISTANCE = 3.0  # meters
    LOOKAHEAD_GAIN = 0.4
    MIN_LOOKAHEAD = 1.8

    # PID Longitudinal Controller
    KP = 1.2
    KI = 0.05
    KD = 0.15

    # LiDAR Sensor Simulation
    LIDAR_NUM_RAYS = 36         # 360-degree sweep at 10-degree increments
    LIDAR_MAX_RANGE = 12.0      # meters
    LIDAR_NOISE_STD = 0.04      # Gaussian noise standard deviation (meters)
    LIDAR_SAFETY_DISTANCE = 2.2 # meters (trigger emergency avoidance)

    # Vision Subsystem
    CAMERA_FOV = 90.0           # degrees
    CAMERA_RANGE = 15.0         # meters

    # Colors (RGB)
    COLOR_BG = (22, 27, 34)
    COLOR_ROAD = (35, 42, 54)
    COLOR_LANE_MARKING = (200, 200, 210)
    COLOR_VEHICLE = (59, 130, 246)
    COLOR_VEHICLE_WHEEL = (20, 20, 25)
    COLOR_OBSTACLE = (239, 68, 68)
    COLOR_DYNAMIC_OBSTACLE = (245, 158, 11)
    COLOR_GOAL = (34, 197, 94)
    COLOR_PATH_GLOBAL = (16, 185, 129)
    COLOR_PATH_TRAIL = (99, 102, 241)
    COLOR_LIDAR_RAY = (147, 197, 253)
    COLOR_LIDAR_HIT = (239, 68, 68)
    COLOR_TEXT = (240, 246, 252)
    COLOR_PANEL_BG = (15, 20, 28)
    COLOR_PANEL_BORDER = (48, 54, 61)
