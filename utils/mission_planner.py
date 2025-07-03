from model.mission import Mission
from model.waypoint import Waypoint
import math

class MissionPlanner:
    def __init__(self, mission: Mission):
        self.mission = mission

    def add_waypoint(self, waypoint: Waypoint):
        valid = True
        if not (-90 <= waypoint.latitude <= 90):
            print(f"Invalid latitude: {waypoint.latitude}. Must be between -90 and 90 degrees.")
            valid = False
        if not (-180 <= waypoint.longitude <= 180):
            print(f"Invalid longitude: {waypoint.longitude}. Must be between -180 and 180 degrees.")
            valid = False
        if valid:
            self.mission.add_waypoint(waypoint)

    def remove_waypoint(self, index: int):
        self.mission.remove_waypoint(index)

    def generate_point_to_point(self, start: Waypoint, end: Waypoint, num_points: int = 5, smooth: bool = False):
        """Generate direct waypoints from start to end at their specified altitudes."""
        start_wp = Waypoint(start.latitude, start.longitude, start.altitude)
        end_wp = Waypoint(end.latitude, end.longitude, end.altitude)
        self.add_waypoint(start_wp)
        self.add_waypoint(end_wp)

    def generate_grid(self, corners: list[Waypoint], altitude: float, spacing: float):
        min_lat = min(wp.latitude for wp in corners)
        max_lat = max(wp.latitude for wp in corners)
        min_lon = min(wp.longitude for wp in corners)
        max_lon = max(wp.longitude for wp in corners)
        for lat in range(int(min_lat * 1000), int(max_lat * 1000 + 1), int(spacing * 1000)):
            for lon in range(int(min_lon * 1000), int(max_lon * 1000 + 1), int(spacing * 1000)):
                wp = Waypoint(lat / 1000, lon / 1000, altitude)
                self.add_waypoint(wp)

    def generate_circle(self, center: Waypoint, radius: float, num_points: int = 10, spiral: bool = False):
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            if spiral:
                r = radius * (i + 1) / num_points
            else:
                r = radius
            lat = center.latitude + (r * math.cos(angle)) / 111.32
            lon = center.longitude + (r * math.sin(angle)) / (111.32 * math.cos(math.radians(center.latitude)))
            wp = Waypoint(lat, lon, center.altitude)
            self.add_waypoint(wp)

    def generate_route_via_landmarks(self, landmarks: list[Waypoint], num_points_per_segment: int = 3):
        for i in range(len(landmarks) - 1):
            self.generate_point_to_point(landmarks[i], landmarks[i + 1], num_points_per_segment)

    def generate_zigzag(self, corners: list[Waypoint], altitude: float, spacing: float, direction: str = "horizontal"):
        min_lat = min(wp.latitude for wp in corners)
        max_lat = max(wp.latitude for wp in corners)
        min_lon = min(wp.longitude for wp in corners)
        max_lon = max(wp.longitude for wp in corners)
        reverse = False
        if direction == "horizontal":
            for lat in range(int(min_lat * 1000), int(max_lat * 1000 + 1), int(spacing * 1000)):
                for lon in range(int(min_lon * 1000), int(max_lon * 1000 + 1), int(spacing * 1000) if not reverse else -int(spacing * 1000)):
                    wp = Waypoint(lat / 1000, lon / 1000, altitude)
                    self.add_waypoint(wp)
                reverse = not reverse
        else:
            for lon in range(int(min_lon * 1000), int(max_lon * 1000 + 1), int(spacing * 1000)):
                for lat in range(int(min_lat * 1000), int(max_lat * 1000 + 1), int(spacing * 1000) if not reverse else -int(spacing * 1000)):
                    wp = Waypoint(lat / 1000, lon / 1000, altitude)
                    self.add_waypoint(wp)
                reverse = not reverse