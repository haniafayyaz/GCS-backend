from typing import List
from model.waypoint import Waypoint

class MissionView:
    def display_mission(self, waypoints: List[Waypoint]):
        print("Current Mission Waypoints:")
        if not waypoints:
            print("  No waypoints defined.")
        for i, wp in enumerate(waypoints):
            print(f"  {i}: {wp}")

    def get_waypoint_input(self, prompt: str = "Enter waypoint: ") -> Waypoint:
        try:
            print(prompt)
            lat = float(input("Enter latitude: "))
            lon = float(input("Enter longitude: "))
            alt = float(input("Enter altitude (meters): "))
            return Waypoint(lat, lon, alt)
        except ValueError:
            print("Invalid input. Using default waypoint.")
            return Waypoint(0.0, 0.0, 10.0)

    def display_initial_choice(self):
        print("\nWelcome to Ground Control Station!")
        print("1. Mission")
        print("2. Free Form")

    def display_mission_types(self):
        print("\nMission Types:")
        print("1. Manual Mission")
        print("2. Point-to-Point Mission")
        print("3. Area Survey (Grid) Mission")
        print("4. Orbit / Circle Mission")
        print("5. Route via Landmarks Mission")
        print("6. Patterned Scan (Lawnmower, Zigzag) Mission")

    def display_manual_options(self):
        print("\nManual Mission Options:")
        print("1. Add Waypoint")
        print("2. Remove Waypoint")
        print("3. Finish and Export")

    def get_shape_corners(self) -> List[Waypoint]:
        corners = []
        num_corners = int(input("Enter number of corners: "))
        for i in range(num_corners):
            print(f"Corner {i + 1}:")
            corners.append(self.get_waypoint_input())
        return corners

    def get_landmarks(self) -> List[Waypoint]:
        landmarks = []
        num_landmarks = int(input("Enter number of landmarks: "))
        for i in range(num_landmarks):
            print(f"Landmark {i + 1}:")
            landmarks.append(self.get_waypoint_input())
        return landmarks

    def display_shape_types(self):
        print("\nFree Form Shape Types:")
        print("1. Polygon (Irregular)")
        print("2. Rectangle")
        print("3. Circle")