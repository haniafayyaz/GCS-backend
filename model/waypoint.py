class Waypoint:
    """Represents a single waypoint in a mission."""
    def __init__(self, latitude: float, longitude: float, altitude: float, command: str = "WAYPOINT"):
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.command = command  # e.g., WAYPOINT, TAKEOFF, LAND

    def __str__(self):
        return f"Waypoint(lat={self.latitude}, lon={self.longitude}, alt={self.altitude}, cmd={self.command})"