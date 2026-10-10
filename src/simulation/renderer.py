import math
import os
import pygame
import cv2
import numpy as np
from typing import List, Tuple, Dict, Any
from config.config import Config
from src.control.vehicle_model import BicycleVehicleModel
from src.perception.lidar import LidarScanResult
from src.planning.local_planner import NavigationState

class SimulationRenderer:
    """
    Renders the autonomous vehicle simulation, LiDAR rays, trajectory overlays,
    telemetry HUD, and embedded computer vision camera feed.
    """

    def __init__(self, screen_width: int = Config.SCREEN_WIDTH, screen_height: int = Config.SCREEN_HEIGHT):
        pygame.init()
        pygame.font.init()
        self.width = screen_width
        self.height = screen_height
        self.scale = Config.SIMULATION_SCALE
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("AI-Based Autonomous Navigation System - Telemetry Dashboard")
        self.clock = pygame.time.Clock()

        self.font_main = pygame.font.SysFont("Segoe UI", 16, bold=True)
        self.font_title = pygame.font.SysFont("Segoe UI", 20, bold=True)
        self.font_small = pygame.font.SysFont("Segoe UI", 13)

        self.show_lidar_rays = True
        self.show_global_path = True

    def m2px(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """Converts meter coordinates to screen pixel coordinates."""
        return (int(x_m * self.scale), int(y_m * self.scale))

    def render_frame(self,
                     vehicle: BicycleVehicleModel,
                     static_obstacles: List[Tuple[float, float, float, float]],
                     dynamic_obstacles: List[Any],
                     goal_pos: Tuple[float, float],
                     global_path: List[Tuple[float, float]],
                     history_trail: List[Tuple[float, float]],
                     lidar_data: LidarScanResult,
                     camera_bgr_frame: np.ndarray,
                     nav_state: NavigationState,
                     min_clearance: float) -> bool:
        """
        Renders full simulation scene.
        Returns False if user closed window, else True.
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_l:
                    self.show_lidar_rays = not self.show_lidar_rays
                elif event.key == pygame.K_p:
                    self.show_global_path = not self.show_global_path

        # 1. Background / Roadway
        self.screen.fill(Config.COLOR_BG)

        # Draw road corridors
        road_rect = pygame.Rect(self.m2px(0.5, 0.5), (int((Config.WORLD_WIDTH_M - 1) * self.scale), int((Config.WORLD_HEIGHT_M - 1) * self.scale)))
        pygame.draw.rect(self.screen, Config.COLOR_ROAD, road_rect)

        # Draw subtle grid/lane guides
        for y_guideline in range(6, int(Config.WORLD_HEIGHT_M), 6):
            py = int(y_guideline * self.scale)
            for x_seg in range(0, self.width, 30):
                pygame.draw.line(self.screen, (50, 60, 75), (x_seg, py), (x_seg + 15, py), 1)

        # 2. Static Obstacles
        for ox, oy, ow, oh in static_obstacles:
            px, py = self.m2px(ox, oy)
            pw, ph = int(ow * self.scale), int(oh * self.scale)
            # Outer block
            pygame.draw.rect(self.screen, (38, 45, 58), (px, py, pw, ph))
            # Border highlight
            pygame.draw.rect(self.screen, (74, 85, 104), (px, py, pw, ph), 2)

        # 3. Dynamic Obstacles
        for dyn in dynamic_obstacles:
            dpx, dpy = self.m2px(dyn.x, dyn.y)
            rad_px = max(6, int(max(dyn.width, dyn.length) * self.scale / 2.0))
            color = Config.COLOR_DYNAMIC_OBSTACLE if dyn.obs_type == "Pedestrian" else (249, 115, 22)
            pygame.draw.circle(self.screen, color, (dpx, dpy), rad_px)
            pygame.draw.circle(self.screen, (255, 255, 255), (dpx, dpy), rad_px, 1)

            # Velocity vector indicator
            v_end = (int(dpx + dyn.vx * 12), int(dpy + dyn.vy * 12))
            pygame.draw.line(self.screen, (255, 255, 255), (dpx, dpy), v_end, 2)

        # 4. Global Planned Path
        if self.show_global_path and len(global_path) > 1:
            pts_px = [self.m2px(wx, wy) for wx, wy in global_path]
            pygame.draw.lines(self.screen, Config.COLOR_PATH_GLOBAL, False, pts_px, 3)
            # Waypoint dots
            for pt in pts_px[::3]:
                pygame.draw.circle(self.screen, (52, 211, 153), pt, 3)

        # 5. Trajectory History Trail
        if len(history_trail) > 1:
            trail_pts = [self.m2px(tx, ty) for tx in history_trail[-150:] for tx, ty in [tx]]
            if len(trail_pts) > 1:
                pygame.draw.lines(self.screen, Config.COLOR_PATH_TRAIL, False, trail_pts, 2)

        # 6. Target Goal Beacon
        gx_px, gy_px = self.m2px(*goal_pos)
        pygame.draw.circle(self.screen, Config.COLOR_GOAL, (gx_px, gy_px), 14, 2)
        pygame.draw.circle(self.screen, (74, 222, 128), (gx_px, gy_px), 8)
        pygame.draw.circle(self.screen, (255, 255, 255), (gx_px, gy_px), 3)

        # 7. LiDAR Rays and Hit Points
        vx_px, vy_px = self.m2px(vehicle.state.x, vehicle.state.y)
        if self.show_lidar_rays and lidar_data:
            for hp in lidar_data.hit_points:
                hx_px, hy_px = self.m2px(*hp)
                # Faint laser ray
                pygame.draw.line(self.screen, (40, 80, 120), (vx_px, vy_px), (hx_px, hy_px), 1)
                # Impact dot
                pygame.draw.circle(self.screen, Config.COLOR_LIDAR_HIT, (hx_px, hy_px), 2)

        # 8. Ego Vehicle
        corners = vehicle.get_bounding_box(Config.VEHICLE_LENGTH, Config.VEHICLE_WIDTH)
        corners_px = [self.m2px(cx, cy) for cx, cy in corners]
        pygame.draw.polygon(self.screen, Config.COLOR_VEHICLE, corners_px)
        pygame.draw.polygon(self.screen, (191, 219, 254), corners_px, 2)

        # Vehicle Direction Headlight
        front_pt = self.m2px(
            vehicle.state.x + math.cos(vehicle.state.yaw) * (Config.VEHICLE_LENGTH / 2.0),
            vehicle.state.y + math.sin(vehicle.state.yaw) * (Config.VEHICLE_LENGTH / 2.0)
        )
        pygame.draw.circle(self.screen, (254, 240, 138), front_pt, 4)

        # 9. Top-Right Embedded Camera Feed
        if camera_bgr_frame is not None:
            cam_rgb = cv2.cvtColor(camera_bgr_frame, cv2.COLOR_BGR2RGB)
            cam_surf = pygame.surfarray.make_surface(np.rot90(np.fliplr(cam_rgb)))
            cam_x = self.width - 335
            cam_y = 15
            # Draw container panel
            pygame.draw.rect(self.screen, Config.COLOR_PANEL_BG, (cam_x - 5, cam_y - 5, 330, 210), border_radius=6)
            pygame.draw.rect(self.screen, Config.COLOR_PANEL_BORDER, (cam_x - 5, cam_y - 5, 330, 210), 2, border_radius=6)
            self.screen.blit(cam_surf, (cam_x, cam_y))

        # 10. Left Telemetry HUD Panel
        self._render_telemetry_hud(vehicle, goal_pos, nav_state, min_clearance)

        pygame.display.flip()
        self.clock.tick(Config.FPS)
        return True

    def _render_telemetry_hud(self, vehicle: BicycleVehicleModel, goal_pos: Tuple[float, float],
                              nav_state: NavigationState, min_clearance: float):
        """Draws real-time instrument panel in top-left."""
        panel_w, panel_h = 280, 210
        panel_x, panel_y = 15, 15

        # Backdrop
        overlay = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        overlay.fill((15, 20, 28, 220))
        self.screen.blit(overlay, (panel_x, panel_y))
        pygame.draw.rect(self.screen, Config.COLOR_PANEL_BORDER, (panel_x, panel_y, panel_w, panel_h), 2, border_radius=6)

        # Title
        title_surf = self.font_title.render("AUTONOMOUS TELEMETRY", True, (96, 165, 250))
        self.screen.blit(title_surf, (panel_x + 12, panel_y + 10))

        # FSM State with color badge
        state_color = (34, 197, 94) if nav_state == NavigationState.CRUISING else \
                      ((245, 158, 11) if nav_state == NavigationState.AVOIDING else \
                      ((239, 68, 68) if nav_state == NavigationState.EMERGENCY_BRAKE else (59, 130, 246)))

        status_text = self.font_main.render(f"FSM: {nav_state.value}", True, state_color)
        self.screen.blit(status_text, (panel_x + 12, panel_y + 40))

        # Speedometer
        speed_kmh = vehicle.state.v * 3.6
        speed_text = self.font_small.render(f"Speed: {speed_kmh:.1f} km/h ({vehicle.state.v:.2f} m/s)", True, Config.COLOR_TEXT)
        self.screen.blit(speed_text, (panel_x + 12, panel_y + 70))

        # Steering angle
        steer_deg = math.degrees(vehicle.state.steer)
        steer_text = self.font_small.render(f"Steering Angle: {steer_deg:+.1f} deg", True, Config.COLOR_TEXT)
        self.screen.blit(steer_text, (panel_x + 12, panel_y + 92))

        # Distance to goal
        dist_goal = math.hypot(goal_pos[0] - vehicle.state.x, goal_pos[1] - vehicle.state.y)
        goal_text = self.font_small.render(f"Dist to Target: {dist_goal:.1f} m", True, Config.COLOR_TEXT)
        self.screen.blit(goal_text, (panel_x + 12, panel_y + 114))

        # Obstacle Clearance
        clear_color = (239, 68, 68) if min_clearance < 2.0 else (200, 220, 240)
        clear_text = self.font_small.render(f"LiDAR Clearance: {min_clearance:.2f} m", True, clear_color)
        self.screen.blit(clear_text, (panel_x + 12, panel_y + 136))

        # Frame rate
        fps = int(self.clock.get_fps())
        fps_text = self.font_small.render(f"FPS: {fps} | Pose: ({vehicle.state.x:.1f}, {vehicle.state.y:.1f})", True, (148, 163, 184))
        self.screen.blit(fps_text, (panel_x + 12, panel_y + 158))

        # Hotkeys
        keys_text = self.font_small.render("[L] LiDAR Rays | [P] Path | [S] Save Shot", True, (100, 116, 139))
        self.screen.blit(keys_text, (panel_x + 12, panel_y + 182))

    def save_screenshot(self, filepath: str):
        """Saves current screen surface to an image file."""
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        pygame.image.save(self.screen, filepath)

    def close(self):
        pygame.quit()
