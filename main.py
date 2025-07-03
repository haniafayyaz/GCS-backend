from pymavlink import mavutil
import time
from model.mission import Mission
from views.display import MissionView
from controller.mission_controller import MissionController
from utils.mission_planner import MissionPlanner
from utils.waypoints_exporter import WaypointsExporter

# === CONFIGURATION ===
TARGET_ALT = 10  # Target Altitude (meters)

def wait_for_heartbeat(master):
    print("⏳ Waiting for heartbeat...")
    master.wait_heartbeat(timeout=15)
    print(f"✅ Heartbeat received! System ID: {master.target_system}, Component ID: {master.target_component}")
    time.sleep(3)

def disable_arming_checks(master):
    print("⚠️ Disabling arming checks (for SITL only)...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        b'ARMING_CHECK',
        0,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(1)

def set_guided_mode(master):
    print("🧭 Setting mode to GUIDED...")
    master.set_mode('GUIDED')
    for i in range(10):
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and hb.custom_mode == 4:
            print("✅ GUIDED mode confirmed.")
            return
        print(f"⏳ Waiting for GUIDED mode... Attempt {i+1}")
        time.sleep(1)
    print("❌ Failed to confirm GUIDED mode.")

def arm_uav(master):
    print("🔐 Arming UAV...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    for i in range(10):
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("✅ UAV is armed.")
            return True
        print(f"🔍 Waiting to arm... Attempt {i+1}")
        time.sleep(1)
    print("❌ Failed to arm UAV.")
    return False

def takeoff(master, altitude):
    print(f"🚀 Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(8)

def goto_location(master, lat, lon, alt):
    print(f"✈️ Flying to lat: {lat}, lon: {lon}, alt: {alt}m...")
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )
    time.sleep(5)  # Adjust based on UAV speed

def main():
    print("🔌 Connecting to SITL (udp:127.0.0.1:14550)...")
    master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

    mission = Mission()
    view = MissionView()
    planner = MissionPlanner(mission)
    exporter = WaypointsExporter()
    controller = MissionController(mission, view, planner, exporter)
    controller.run()

    try:
        wait_for_heartbeat(master)
        disable_arming_checks(master)
        set_guided_mode(master)

        if not arm_uav(master):
            print("❌ Aborting: UAV failed to arm.")
            exit(1)

        waypoints = mission.get_waypoints()
        if waypoints:
            takeoff(master, waypoints[0].altitude)
            for wp in waypoints:
                goto_location(master, wp.latitude, wp.longitude, wp.altitude)
        else:
            print("No waypoints defined. Using default target.")
            takeoff(master, TARGET_ALT)
            goto_location(master, -35.3620, 149.1652, TARGET_ALT)

        print("✅ Mission completed.")
    except Exception as e:
        print("❌ ERROR:", e)

if __name__ == "__main__":
    main()