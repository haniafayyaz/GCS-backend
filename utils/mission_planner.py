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

    def point_in_polygon(self, point: Waypoint, polygon: list[Waypoint]) -> bool:
        """Check if a point lies inside a polygon using ray-casting algorithm."""
        x, y = point.longitude, point.latitude
        n = len(polygon)
        inside = False
        j = n - 1
        for i in range(n):
            if ((polygon[i].latitude > y) != (polygon[j].latitude > y)) and \
               (x < (polygon[j].longitude - polygon[i].longitude) * (y - polygon[i].latitude) /
                (polygon[j].latitude - polygon[i].latitude) + polygon[i].longitude):
                inside = not inside
            j = i
        return inside

    def generate_grid(self, corners: list[Waypoint], altitude: float, spacing: float):
        """Generate grid waypoints within a polygon defined by corners."""
        if len(corners) < 3:
            print("Error: At least 3 corners are required to define a polygon.")
            return

        # Calculate bounding box
        min_lat = min(wp.latitude for wp in corners)
        max_lat = max(wp.latitude for wp in corners)
        min_lon = min(wp.longitude for wp in corners)
        max_lon = max(wp.longitude for wp in corners)

        # Convert spacing from km to degrees (approximate, 1 degree ~ 111 km)
        spacing_deg = spacing / 111.0

        # Generate grid points within bounding box
        lat = min_lat
        while lat <= max_lat:
            lon = min_lon
            while lon <= max_lon:
                wp = Waypoint(lat, lon, altitude)
                # Check if waypoint is inside the polygon
                if self.point_in_polygon(wp, corners):
                    self.add_waypoint(wp)
                lon += spacing_deg
            lat += spacing_deg


    def generate_circle(self, center: Waypoint, radius: float, num_points: int = 10, spiral: bool = False):
        """Generate circular or spiral waypoints around a center point."""
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            if spiral:
                r = radius * (i + 1) / num_points
            else:
                r = radius
            lat = center.latitude + (r * math.cos(angle)) / 111.32  # Approx degrees per km
            lon = center.longitude + (r * math.sin(angle)) / (111.32 * math.cos(math.radians(center.latitude)))
            wp = Waypoint(lat, lon, center.altitude)
            self.add_waypoint(wp)

    def generate_route_via_landmarks(self, landmarks: list[Waypoint], num_points_per_segment: int = 3):
        """Generate waypoints connecting a list of landmarks."""
        for i in range(len(landmarks) - 1):
            self.generate_point_to_point(landmarks[i], landmarks[i + 1], num_points_per_segment)

    def point_inside_polygon(self, point: Waypoint, polygon: list[Waypoint]) -> bool:
            """Check if a point lies inside a polygon using ray-casting algorithm."""
            if len(polygon) < 3:
                return False
            x, y = point.longitude, point.latitude
            n = len(polygon)
            inside = False
            j = n - 1
            for i in range(n):
                if abs(polygon[j].latitude - polygon[i].latitude) < 1e-10:
                    j = i
                    continue
                if ((polygon[i].latitude > y) != (polygon[j].latitude > y)) and \
                (x < (polygon[j].longitude - polygon[i].longitude) * (y - polygon[i].latitude) /
                    (polygon[j].latitude - polygon[i].latitude) + polygon[i].longitude):
                    inside = not inside
                j = i
            return inside

    def generate_zigzag(self, corners: list[Waypoint], altitude: float, spacing: float, direction: str = "horizontal"):
        """Generate waypoints in a lawnmower/zigzag pattern within a polygon."""
        self.waypoints = []
        min_lat = min(wp.latitude for wp in corners)
        max_lat = max(wp.latitude for wp in corners)
        min_lon = min(wp.longitude for wp in corners)
        max_lon = max(wp.longitude for wp in corners)
        spacing_deg = spacing / 111  # Convert km to degrees (approx. 111 km per degree)

        reverse = False
        if direction == "horizontal":
            lat = min_lat
            while lat <= max_lat + 1e-10:  # Include max_lat with small epsilon
                # Determine longitude range and direction
                if reverse:
                    lon_start, lon_end = max_lon, min_lon
                    lon_step = -spacing_deg
                else:
                    lon_start, lon_end = min_lon, max_lon
                    lon_step = spacing_deg
                
                lon = lon_start
                while (lon_step > 0 and lon <= lon_end + 1e-10) or (lon_step < 0 and lon >= lon_end - 1e-10):
                    point = Waypoint(lat, lon, altitude)
                    if self.point_inside_polygon(point, corners):
                        self.add_waypoint(point)
                    lon += lon_step
                lat += spacing_deg
                reverse = not reverse
        else:  # vertical
            lon = min_lon
            while lon <= max_lon + 1e-10:
                if reverse:
                    lat_start, lat_end = max_lat, min_lat
                    lat_step = -spacing_deg
                else:
                    lat_start, lat_end = min_lat, max_lat
                    lat_step = spacing_deg
                
                lat = lat_start
                while (lat_step > 0 and lat <= lat_end + 1e-10) or (lat_step < 0 and lat >= lat_end - 1e-10):
                    point = Waypoint(lat, lon, altitude)
                    if self.point_in_polygon(point, corners):
                        self.add_waypoint(point)
                    lat += lat_step
                lon += spacing_deg
                reverse = not reverse