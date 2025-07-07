from typing import List
from .waypoint import Waypoint

class Mission:
    """Represents a mission consisting of multiple waypoints."""
    def __init__(self):
        self.waypoints: List[Waypoint] = []

    def add_waypoint(self, waypoint: Waypoint):
        """Add a waypoint to the mission."""
        self.waypoints.append(waypoint)

    def remove_waypoint(self, index: int):
        """Remove a waypoint at the given index."""
        if 0 <= index < len(self.waypoints):
            self.waypoints.pop(index)

    def get_waypoints(self) -> List[Waypoint]:
        """Return the list of waypoints."""
        return self.waypoints