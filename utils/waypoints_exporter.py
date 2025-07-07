from model.mission import Mission

class WaypointsExporter:
    """Exports mission waypoints to .txt files in the missions folder."""
    def export(self, mission: Mission, filename: str):
        """Export mission to a .txt file in QGroundControl format."""
        try:
            with open(filename, 'w') as f:
                f.write("QGC WPL 110\n")  # QGroundControl waypoints file header
                for i, wp in enumerate(mission.get_waypoints()):
                    # Format: index, currentWP, coordFrame, command, param1-4, autocontinue
                    f.write(f"{i}\t0\t3\t16\t{wp.altitude}\t0\t0\t{wp.latitude}\t{wp.longitude}\t0\t0\t1\n")
        except PermissionError:
            raise PermissionError(f"Permission denied: Cannot write to {filename}")
        except IOError as e:
            raise IOError(f"IO error writing to {filename}: {e}")