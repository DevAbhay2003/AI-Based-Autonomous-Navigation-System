import math
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any
from config.config import Config

class DetectedObject:
    """Represents an object detected by the computer vision subsystem."""

    def __init__(self, label: str, confidence: float, bbox: Tuple[int, int, int, int], distance_m: float):
        self.label = label
        self.confidence = confidence
        self.bbox = bbox          # (x, y, w, h) in camera pixels
        self.distance_m = distance_m

class VisionSubsystem:
    """
    Simulates front-facing camera perception, lane tracking,
    and synthetic bounding-box object detection.
    """

    def __init__(self, fov_deg: float = Config.CAMERA_FOV, max_range: float = Config.CAMERA_RANGE):
        self.fov_rad = math.radians(fov_deg)
        self.max_range = max_range
        self.cam_width = 320
        self.cam_height = 200

    def generate_camera_frame(self, ego_x: float, ego_y: float, ego_yaw: float,
                              obstacles: List[Dict[str, Any]]) -> Tuple[np.ndarray, List[DetectedObject], float]:
        """
        Synthesizes a front camera view with roadway perspective, detected lanes,
        and projected obstacle bounding boxes.
        
        Returns:
            frame: OpenCV BGR image (320x200)
            detected_objects: List of DetectedObject instances
            lane_offset_m: Lateral distance from lane center
        """
        # Create base synthetic camera canvas (Sky + Ground)
        frame = np.zeros((self.cam_height, self.cam_width, 3), dtype=np.uint8)
        frame[0:80, :] = [60, 45, 35]       # Horizon / sky gradient
        frame[80:, :] = [45, 45, 50]        # Road surface asphalt

        # Synthesize perspective lane markings
        lane_horizon_y = 85
        center_x = self.cam_width // 2
        lane_width_bottom = 180
        lane_width_top = 30

        # Draw left and right lane boundaries
        left_pt_bottom = (center_x - lane_width_bottom // 2, self.cam_height)
        left_pt_top = (center_x - lane_width_top // 2, lane_horizon_y)
        right_pt_bottom = (center_x + lane_width_bottom // 2, self.cam_height)
        right_pt_top = (center_x + lane_width_top // 2, lane_horizon_y)

        # Draw road lane boundaries (Yellow solid left, white dashed right)
        cv2.line(frame, left_pt_bottom, left_pt_top, (0, 215, 255), 3) # Yellow line
        cv2.line(frame, right_pt_bottom, right_pt_top, (220, 220, 220), 3) # White line

        # Draw center dashed guide line
        for step in range(5):
            t1 = step / 5.0
            t2 = (step + 0.6) / 5.0
            y1 = int(lane_horizon_y + t1 * (self.cam_height - lane_horizon_y))
            y2 = int(lane_horizon_y + t2 * (self.cam_height - lane_horizon_y))
            cv2.line(frame, (center_x, y1), (center_x, y2), (255, 255, 255), 2)

        # Detect obstacles in camera FOV and project into camera pixel coordinates
        detected_objects: List[DetectedObject] = []
        half_fov = self.fov_rad / 2.0

        for obs in obstacles:
            ox = obs["x"]
            oy = obs["y"]
            dx = ox - ego_x
            dy = oy - ego_y
            dist = math.hypot(dx, dy)

            if dist > self.max_range or dist < 0.5:
                continue

            # Calculate relative angle in vehicle heading coordinate frame
            angle_world = math.atan2(dy, dx)
            rel_angle = angle_world - ego_yaw
            rel_angle = (rel_angle + math.pi) % (2.0 * math.pi) - math.pi

            if -half_fov <= rel_angle <= half_fov:
                # Project onto camera horizontal axis
                norm_x = (rel_angle + half_fov) / self.fov_rad  # [0.0, 1.0]
                px = int(norm_x * self.cam_width)

                # Perspective scaling: objects closer appear lower and larger
                dist_factor = max(0.1, 1.0 - (dist / self.max_range))
                py = int(lane_horizon_y + dist_factor * (self.cam_height - lane_horizon_y - 20))
                box_w = max(16, int(70 * dist_factor))
                box_h = max(16, int(60 * dist_factor))

                bx = max(2, min(self.cam_width - box_w - 2, px - box_w // 2))
                by = max(lane_horizon_y, min(self.cam_height - box_h - 2, py - box_h // 2))

                label = obs.get("type", "Obstacle")
                conf = round(0.85 + 0.12 * dist_factor, 2)
                detected_objects.append(DetectedObject(label, conf, (bx, by, box_w, box_h), round(dist, 2)))

                # Draw bounding box and label overlay on camera frame
                color = (0, 165, 255) if label == "Vehicle" else ((0, 255, 255) if label == "Pedestrian" else (0, 0, 255))
                cv2.rectangle(frame, (bx, by), (bx + box_w, by + box_h), color, 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (bx, max(12, by - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
                cv2.putText(frame, f"{dist:.1f}m", (bx, by + box_h + 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 200), 1, cv2.LINE_AA)

        # Cross-track lane offset estimate (simulated)
        lane_offset_m = round(math.sin(ego_yaw) * 0.25, 2)

        # HUD on camera
        cv2.putText(frame, "CAM-FRONT 60FPS [AI-PERCEPTION]", (6, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(frame, f"LANE DEV: {lane_offset_m:+.2f}m", (6, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (180, 220, 255), 1, cv2.LINE_AA)

        return frame, detected_objects, lane_offset_m
