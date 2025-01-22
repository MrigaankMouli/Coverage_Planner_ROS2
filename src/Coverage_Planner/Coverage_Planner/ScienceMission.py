import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Vector3
from pymavlink import mavutil
import time
import threading
from ament_index_python.packages import get_package_share_directory
import os
import json
import os

def monitor_thread_func(controller):
    """
    Monitor Thread that gives control to Safety Pilot by killing the script

    The Safety Pilot is only given control if the Flight Mode is POSHOLD 
    and RC Channel 3(Mapped to Throttle) has a PWM value of above 1350

    """

    print("Monitor Thread Running")

    while True:
        msg = controller.recv_match(type=["RC_CHANNELS","HEARTBEAT"],blocking = True)
        if not msg:
            continue

        if msg.get_type() == 'HEARTBEAT':
            mode = mavutil.mode_string_v10(msg)
            
            if mode == "POSHOLD":
                print("Mode set to POSHOLD. Checking Throttle Value...")

                rc_msg = controller.recv_match(type = "RC_CHANNELS",blocking = True)
                if rc_msg and rc_msg.chan3_raw>1350:
                    print("Pilot taking over control. Exiting Script.")
                    os._exit(0)
                else:
                    print("Throttle value too low. Control remains with the script")
        
        time.sleep(0.1)

def monitor_waypoint_progress(controller):
    """
    Monitors progress of the entire mission.

    """
    print("Starting waypoint monitoring...")

    controller.mav.mission_request_list_send(
        controller.target_system,
        controller.target_component
    )
    msg = controller.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
    if not msg:
        print("Failed to get the total number of waypoints.")
        return

    total_waypoints = msg.count
    print(f"Total waypoints: {total_waypoints}")

    while True:
        reached_msg = controller.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        if reached_msg:
            if reached_msg.seq == 0:
                continue

            print(f"Waypoint {reached_msg.seq} reached!")

            if reached_msg.seq == total_waypoints :
                print("Final waypoint reached. Mission complete.")
                break



class MissionItem:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1 #AutoContinue to next WP(0 for false, 1 for true)
        self.param1 = 3.0 #Hold Time in Seconds
        self.param2 = 0.5 #Acceptance Radius in Metres
        self.param3 = 0.0 #Pass Radius(1 to Loiter at the WP for duration specified in Param1 and 0 to Pass through the WP)
        self.param4 = 0.0 #Desired Yaw Angle at the WP
        self.x = int(x) #Latitude in Lat*1e7 format
        self.y = int(y) #Longitude in Lon*1e7 format
        self.z = float(z) #Relative Altitude
        self.mission_type = 0

def set_mode(controller, mode):
    """
    Setting the Flight Mode of the Drone
    3:Auto
    4:Guided
    5:Loiter
    6:RTL
    9:Land
    16:PosHold

    """
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
            0, 0, 0, 0, 0
        )
        print(f"Mode set to {mode}")
        return True
    except Exception as e:
        print(f"Failed to set mode. Error: {e}")
        return False

def set_speed(controller, speed, speed_type):
    """
    Setting the speed of the Drone
    Speed Types-
    0:Air Speed
    1:Ground Speed
    2:Climb Speed
    3:Descent Speed

    """
    print(f"Setting speed to {speed} of type {speed_type}")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        speed_type,
        speed,
        -1,
        0, 0, 0, 0
    )
    msg = controller.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"Failed to set speed: {msg.result}")
    else:
        print(f"Speed set successfully.")

def load_home_position(controller):
    """
    Setting the Starting(Home) Position of the Drone
    as the Arming Position
    The Drone will Takeoff and Land at this Position

    """
    print("Loading Starting position...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        0, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = controller.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            print(f"Home position loaded: {msg.lat}, {msg.lon}")
            return (msg.lat, msg.lon, 0)

def load_lap_waypoints(file_path):
    """
    Loading Waypoints from JSON file
    Latitude is changed to lat*1e7 format
    Longitude is changed to lon*1e7 format

    """
    print(f"Loading lap waypoints from {file_path}...")
    with open(file_path, 'r') as f:
        data = json.load(f)
        return [{"lat": wp["latitude"], "lon": wp["longitude"]} for wp in data.get("lap_waypoints", [])]

def upload_mission(controller, home_pos, vertices):
    """
    Uploading Mission to Flight Controller
    Takeoff and Land is at Arming Position
    Rest of the Waypoints are loaded from the JSON file

    """
    print("Uploading mission...")
    controller.mav.mission_clear_all_send(controller.target_system, controller.target_component)
    time.sleep(1)
    mission_items = [MissionItem(0, current=1, x=home_pos[0], y=home_pos[1], z=7)]
    mission_items[0].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    print("Takeoff waypoint added.")

    for i, vertex in enumerate(vertices, start=1):
        mission_items.append(MissionItem(i, current=1, x=vertex['lat'], y=vertex['lon'], z=7))
        print(f"Waypoint {i} added: {vertex['lat']}, {vertex['lon']}")

    mission_items.append(MissionItem(len(vertices)+1, current=0, x=home_pos[0], y=home_pos[1], z=0))
    mission_items[-1].command = mavutil.mavlink.MAV_CMD_NAV_LAND
    print("Landing waypoint added.")

    controller.mav.mission_count_send(controller.target_system, controller.target_component, len(mission_items), 0)
    time.sleep(1)
    for item in mission_items:
        controller.mav.mission_item_int_send(
            controller.target_system,
            controller.target_component,
            item.seq,
            item.frame,
            item.command,
            item.current,
            item.auto,
            item.param1,
            item.param2,
            item.param3,
            item.param4,
            item.x,
            item.y,
            item.z,
            item.mission_type
        )
        time.sleep(0.001)
        print(f"Mission item {item.seq} uploaded.")

def arm_drone(controller):
    """
    Arming the Drone
    
    """
    print("Arming drone...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0, 0, 0, 0, 0
    )
    # time.sleep(2)
    print("Drone armed.")

def takeoff_drone(controller, altitude):
    """
    Taking off to Set Altitude
    
    """
    print(f"Taking off to {altitude} meters...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0, 0, 0, 0, altitude
    )

def disarm_drone(controller):
    """
    Disarming the Drone

    """
    print("Disarming drone...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0,
        0,
        0, 0, 0, 0, 0
    )
    time.sleep(2)
    print("Drone disarmed.")

def start_mission(controller):
    """
    Starting the Mission
    Mode is automatically set to AUTO(It should but for some reason we need to manually set it for Cleo)
    
    """
    print("Starting mission...")
    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def main(args=None):
    print("Initializing connection...")
    controller = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    controller.wait_heartbeat()
    print("Connection established.")
    
    monitor_thread = threading.Thread(target=monitor_thread_func, args=(controller,), daemon=True)
    monitor_thread.start()

    gps_msg = controller.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if gps_msg:
        current_lat = gps_msg.lat/1e7  
        current_lon = gps_msg.lon/1e7

        relative_alt = gps_msg.relative_alt/1000
    
    controller.mav.command_long_send(
        controller.target_system,        
        controller.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  
        0,                               
        1,                               
        0,                               
        0,                               
        0,                               
        current_lat,
        current_lon,                     
        relative_alt
    )

    print("Setting Mode to Guided")
    set_mode(controller, mode=4)
    time.sleep(1)

    home_pos = load_home_position(controller)
    package_share_dir = get_package_share_directory("Coverage_Planner")
    lap_waypoints = load_lap_waypoints(os.path.join(package_share_dir, 'Waypoints')+"/coverage_boundary.json")
    upload_mission(controller, home_pos, lap_waypoints)
    arm_drone(controller)
    time.sleep(2)
    takeoff_drone(controller, altitude=3.048)
    time.sleep(8) 
    set_mode(controller, mode=3)
    time.sleep(2)
    start_mission(controller)

    print("Starting waypoint monitoring...")

    controller.mav.mission_request_list_send(
        controller.target_system,
        controller.target_component
    )
    msg = controller.recv_match(type='MISSION_COUNT', blocking=False)

    while not msg:
        print("Failed to get the total number of waypoints.")
        msg = controller.recv_match(type='MISSION_COUNT', blocking=False)

    total_waypoints = msg.count
    print(f"Total waypoints: {total_waypoints}")

    mission_end_flag = 0

    while True:
        reached_msg = controller.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        if reached_msg:
            if reached_msg.seq == 0:
                continue

            print(f"Waypoint {reached_msg.seq} reached!")

            if reached_msg.seq == 1 and mission_end_flag == 1 :
                print("Final waypoint reached. Mission complete.")
                break
        
            if reached_msg.seq == 1 and mission_end_flag == 0 :
                mission_end_flag = 1

    
