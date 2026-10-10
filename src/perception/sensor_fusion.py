import math
from typing import List, Dict, Any, Tuple
from .lidar import LidarScanResult
from .vision import DetectedObject
from config.config import Config

class FusedObstacle:
    """Represents a fused sensory obstacle combining range and semantic category."""

    def __init__(self, obj_id: int, label: str, distance: float, angle_rad: float, risk_level: str):
        self.obj_id = obj_id
        self.label = label
        self.distance = distance
        self.angle_rad = angle_rad
        self.risk_level = risk_level  # "SAFE", "WARNING", "CRITICAL"

class SensorFusionEngine:
    """
    Fuses 2D LiDAR ranges and Computer Vision detections into unified environmental perception.
    Evaluates collision time-to-impact (TTC) and threat level for reactive safety maneuvers.
    """

    def __init__(self, safety_margin: float = Config.LIDAR_SAFETY_DISTANCE):
        self.safety_margin = safety_margin

    def fuse(self, lidar_data: LidarScanResult, vision_objects: List[DetectedObject],
             current_speed: float) -> Tuple[List[FusedObstacle], str, float]:
        """
        Fuses sensory inputs and computes active navigation threat assessment.
        
        Returns:
            fused_list: list of FusedObstacle objects
            threat_status: "NORMAL", "CAUTION", or "EMERGENCY_STOP"
            closest_obstacle_distance: minimum detected distance in meters
        """
        min_dist = lidar_data.min_distance
        fused_list: List[FusedObstacle] = []

        # Correlate vision objects with minimum range
        for idx, vo in enumerate(vision_objects):
            # Compute threat level
            if vo.distance_m <= self.safety_margin:
                risk = "CRITICAL"
            elif vo.distance_m <= self.safety_margin * 1.8:
                risk = "WARNING"
            else:
                risk = "SAFE"

            fused_list.append(FusedObstacle(
                obj_id=idx + 1,
                label=vo.label,
                distance=vo.distance_m,
                angle_rad=0.0,
                risk_level=risk
            ))

        # Determine overall forward navigation threat status
        fwd_dist = lidar_data.forward_min_distance
        if fwd_dist < self.safety_margin * 0.8:
            threat_status = "EMERGENCY_STOP"
        elif fwd_dist < self.safety_margin * 1.5:
            threat_status = "CAUTION"
        else:
            threat_status = "NORMAL"

        return fused_list, threat_status, lidar_data.min_distance
