import os
from model.mission import Mission
from views.display import MissionView
from utils.mission_planner import MissionPlanner
from utils.waypoints_exporter import WaypointsExporter

class MissionController:
    def __init__(self, mission: Mission, view: MissionView, planner: MissionPlanner, exporter: WaypointsExporter):
        self.mission = mission
        self.view = view
        self.planner = planner
        self.exporter = exporter
        self.mission_counter = 1
        self.missions_dir = "missions"
        os.makedirs(self.missions_dir, exist_ok=True)

    def get_unique_filename(self) -> str:
        while os.path.exists(os.path.join(self.missions_dir, f"mission{self.mission_counter}.txt")):
            self.mission_counter += 1
        return os.path.join(self.missions_dir, f"mission{self.mission_counter}.txt")

    def run(self):
        self.view.display_initial_choice()
        initial_choice = input("Select option (1 for Mission, 2 for Free Form): ")
        if initial_choice == "1":
            self.view.display_mission_types()
            mission_type = input("Select mission type (1-6): ")
            if mission_type == "1":  # Manual Mission
                while True:
                    self.view.display_manual_options()
                    manual_choice = input("Select option (1-3): ")
                    if manual_choice == "1":
                        waypoint = self.view.get_waypoint_input()
                        self.planner.add_waypoint(waypoint)
                    elif manual_choice == "2":
                        try:
                            index = int(input("Enter waypoint index to remove: "))
                            self.planner.remove_waypoint(index)
                        except ValueError:
                            print("Invalid index.")
                    elif manual_choice == "3":
                        break
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            # Add other mission types (2-6) as in your original code...
            elif mission_type == "2":  # Point-to-Point
                start = self.view.get_waypoint_input("Enter start waypoint: ")
                end = self.view.get_waypoint_input("Enter end waypoint: ")
                self.planner.generate_point_to_point(start, end)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "3":  # Area Survey (Grid)
                corners = self.view.get_shape_corners()
                altitude = float(input("Enter altitude (meters): "))
                spacing = float(input("Enter spacing (km): "))
                self.planner.generate_grid(corners, altitude, spacing)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "4":  # Orbit / Circle
                center = self.view.get_waypoint_input("Enter center waypoint: ")
                radius = float(input("Enter radius (km): "))
                num_points = int(input("Enter number of points: ") or 10)
                spiral = input("Spiral path? (y/n): ").lower() == 'y'
                self.planner.generate_circle(center, radius, num_points, spiral)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "5":  # Route via Landmarks
                landmarks = self.view.get_landmarks()
                num_points = int(input("Enter points per segment: ") or 3)
                self.planner.generate_route_via_landmarks(landmarks, num_points)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "6":  # Patterned Scan (Zigzag)
                corners = self.view.get_shape_corners()
                altitude = float(input("Enter altitude (meters): "))
                spacing = float(input("Enter spacing (km): "))
                direction = input("Scan direction (horizontal/vertical): ").lower() or "horizontal"
                self.planner.generate_zigzag(corners, altitude, spacing, direction)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
        elif initial_choice == "2":
            self.view.display_shape_types()
            shape_choice = input("Select shape type (1-3): ")
            print("Free Form option selected. Functionality to be implemented.")
        else:
            print("Invalid choice. Please select 1 or 2.")
        print("Mission completed and stored. Exiting...")
import os
from model.mission import Mission
from views.display import MissionView
from utils.mission_planner import MissionPlanner
from utils.waypoints_exporter import WaypointsExporter

class MissionController:
    def __init__(self, mission: Mission, view: MissionView, planner: MissionPlanner, exporter: WaypointsExporter):
        self.mission = mission
        self.view = view
        self.planner = planner
        self.exporter = exporter
        self.mission_counter = 1
        self.missions_dir = "missions"
        os.makedirs(self.missions_dir, exist_ok=True)

    def get_unique_filename(self) -> str:
        while os.path.exists(os.path.join(self.missions_dir, f"mission{self.mission_counter}.txt")):
            self.mission_counter += 1
        return os.path.join(self.missions_dir, f"mission{self.mission_counter}.txt")

    def run(self):
        self.view.display_initial_choice()
        initial_choice = input("Select option (1 for Mission, 2 for Free Form): ")
        if initial_choice == "1":
            self.view.display_mission_types()
            mission_type = input("Select mission type (1-6): ")
            if mission_type == "1":  # Manual Mission
                while True:
                    self.view.display_manual_options()
                    manual_choice = input("Select option (1-3): ")
                    if manual_choice == "1":
                        waypoint = self.view.get_waypoint_input()
                        self.planner.add_waypoint(waypoint)
                    elif manual_choice == "2":
                        try:
                            index = int(input("Enter waypoint index to remove: "))
                            self.planner.remove_waypoint(index)
                        except ValueError:
                            print("Invalid index.")
                    elif manual_choice == "3":
                        break
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            # Add other mission types (2-6) as in your original code...
            elif mission_type == "2":  # Point-to-Point
                #start = self.view.get_waypoint_input("Enter start waypoint: ")
                end = self.view.get_waypoint_input("Enter end waypoint: ")
                #self.planner.generate_point_to_point(start, end)
                self.planner.add_waypoint(end)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "3":  # Area Survey (Grid)
                corners = self.view.get_shape_corners()
                altitude = float(input("Enter altitude (meters): "))
                spacing = float(input("Enter spacing (km): "))
                self.planner.generate_grid(corners, altitude, spacing)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "4":  # Orbit / Circle
                center = self.view.get_waypoint_input("Enter center waypoint: ")
                radius = float(input("Enter radius (km): "))
                num_points = int(input("Enter number of points: ") or 10)
                spiral = input("Spiral path? (y/n): ").lower() == 'y'
                self.planner.generate_circle(center, radius, num_points, spiral)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "5":  # Route via Landmarks
                landmarks = self.view.get_landmarks()
                num_points = int(input("Enter points per segment: ") or 3)
                self.planner.generate_route_via_landmarks(landmarks, num_points)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
            elif mission_type == "6":  # Patterned Scan (Zigzag)
                corners = self.view.get_shape_corners()
                altitude = float(input("Enter altitude (meters): "))
                spacing = float(input("Enter spacing (km): "))
                direction = input("Scan direction (horizontal/vertical): ").lower() or "horizontal"
                self.planner.generate_zigzag(corners, altitude, spacing, direction)
                self.view.display_mission(self.mission.get_waypoints())
                filename = self.get_unique_filename()
                self.exporter.export(self.mission, filename)
                print(f"Mission exported to {filename}")
                self.mission_counter += 1
        elif initial_choice == "2":
            self.view.display_shape_types()
            shape_choice = input("Select shape type (1-3): ")
            print("Free Form option selected. Functionality to be implemented.")
        else:
            print("Invalid choice. Please select 1 or 2.")
        print("Mission completed and stored. Exiting...")