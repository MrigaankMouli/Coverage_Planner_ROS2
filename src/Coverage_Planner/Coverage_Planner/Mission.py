import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Vector3
from pymavlink import mavutil
import time
import threading
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation as R
import os
import json


class MissionTimer:
    def __init__(self):
        self.start_time = None
        self.is_running = False
        self.elapsed_time = 0

    def start(self):
        self.start_time = time.time()
        self.is_running = True

    def stop(self):
        if self.is_running:
            self.elapsed_time = time.time() - self.start_time
            self.is_running = False

    def get_elapsed_time(self):
        if self.is_running:
            return time.time() - self.start_time
        return self.elapsed_time

def timer_thread_func(mission_timer):
    """
    Thread that continuously updates and displays mission time
    """
    print("Timer Thread Running")
    while True:
        if mission_timer.is_running:
            elapsed = mission_timer.get_elapsed_time()
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            print(f"\rMission Time: {minutes:02d}:{seconds:02d}", end="", flush=True)
        time.sleep(1)

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

class MissionItem:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1 #AutoContinue to next WP(0 for false, 1 for true)
        self.param1 = 0.0 #Hold Time in Seconds
        self.param2 = 0.5 #Acceptance Radius in Metres
        self.param3 = 0.0 #Pass Radius(1 to Loiter at the WP for duration specified in Param1 and 0 to Pass through the WP)
        self.param4 = 0.0 #Desired Yaw Angle at the WP
        self.x = int(x) #Latitude in Lat*1e7 format
        self.y = int(y) #Longitude in Lon*1e7 format
        self.z = int(z) #Relative Altitude
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
        return [{"lat": wp["latitude"]*1e7, "lon": wp["longitude"]*1e7} for wp in data.get("lap_waypoints", [])]

def upload_mission(controller, home_pos, vertices, altitude):
    """
    Uploading Mission to Flight Controller
    Takeoff and Land is at Arming Position
    Rest of the Waypoints are loaded from the JSON file

    """
    print("Uploading mission...")
    controller.mav.mission_clear_all_send(controller.target_system, controller.target_component)
    time.sleep(1)
    mission_items = [MissionItem(0, current=1, x=home_pos[0], y=home_pos[1], z=altitude)]
    mission_items[0].command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
    print("Takeoff waypoint added.")

    for i, vertex in enumerate(vertices, start=1):
        mission_items.append(MissionItem(i, current=0, x=vertex['lat'], y=vertex['lon'], z=altitude))
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
    time.sleep(2)
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

# SERIAL_PORT_CUBEPILOT = "/dev/ttyACM0"

from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Vector3
from scipy.spatial.transform import Rotation as R

class CubePilotOdometryNode(Node):
    """
    Node to Publish the Odometry, IMU, and GPS data from the GPS and the CubeOrange IMU as ROS2 messages.
    """

    def __init__(self, controller):
        super().__init__('cube_pilot_odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom_cube', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps_cube', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_cube', 10)
        
        self.CubePilot = controller

        self.timer = self.create_timer(0.5, self.publish_odom)
        
    def publish_odom(self):

        self.CubePilot.mav.request_data_stream_send(
            self.CubePilot.target_system,
            self.CubePilot.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  
            30,  
            1
            )

        msg = self.CubePilot.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        msg1 = self.CubePilot.recv_match(type='ATTITUDE', blocking=False)
        
        if msg:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom_frame'
            odom.pose.pose.position = Point(x=msg.x, y=msg.y, z=msg.z)
            odom.twist.twist.linear = Vector3(
                x=msg.vx,
                y=msg.vy,
                z=-1*msg.vz
            )
            
            if msg1:
                odom.twist.twist.angular = Vector3(
                    x=msg1.roll,
                    y=msg1.pitch,
                    z=msg1.yaw
                )
            else:
                self.get_logger().warn("ATTITUDE message not available, skipping angular velocities.")
            
            self.odom_pub.publish(odom)
            self.get_logger().info(f"Published Odometry: X={msg.x}, Y={msg.y}, Z={msg.z}, Vx={msg.vx}, Vy={msg.vy}, Vz={msg.vz}")
        else:
            self.get_logger().warn("LOCAL_POSITION_NED message not available, skipping odometry.")

        msg2 = self.CubePilot.recv_match(type="GPS_RAW_INT", blocking=False)
        
        if msg2:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps_link'
            gps_msg.latitude = msg2.lat / 1e7
            gps_msg.longitude = msg2.lon / 1e7
            gps_msg.altitude = msg2.alt / 1000.0
            gps_msg.status.status = msg2.fix_type
            
            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Published GPS data: Latitude={gps_msg.latitude}, Longitude={gps_msg.longitude}, Altitude={gps_msg.altitude}")
        else:
            self.get_logger().warn("GPS_RAW_INT message not available, skipping GPS coordinates.")

        msg3 = self.CubePilot.recv_match(type='RAW_IMU', blocking=False)
        print(msg3)

        if msg3:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = float(msg3.xacc)
            imu_msg.linear_acceleration.y = float(msg3.yacc)
            imu_msg.linear_acceleration.z = float(msg3.zacc)

            imu_msg.angular_velocity.x = float(msg3.xgyro)
            imu_msg.angular_velocity.y = float(msg3.ygyro)
            imu_msg.angular_velocity.z = float(msg3.zgyro)

            if msg1:
                r = R.from_euler('xyz', [msg1.roll, msg1.pitch, msg1.yaw], degrees=False)
                quaternion = r.as_quat()
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]
            # else:
            #     imu_msg.orientation.x = 0.0
            #     imu_msg.orientation.y = 0.0
            #     imu_msg.orientation.z = 0.0
            #     imu_msg.orientation.w = 1.0
            
            self.imu_pub.publish(imu_msg)
            self.get_logger().info("Published IMU data with orientation quaternion.")
        else:
            self.get_logger().warn("SCALED_IMU2 message not available, skipping IMU data.")


def PubThread_Func(controller):
    """
    Thread to start the Odometry, IMU and GPS data Publisher

    """
    rclpy.init(args=None)
    cube_pilot_node = CubePilotOdometryNode(controller)

    rclpy.spin(cube_pilot_node)


def main(args=None):
    print("Initializing connection...")
    controller = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    controller.wait_heartbeat()
    print("Connection established.")
    
    mission_timer = MissionTimer()
    
    monitor_thread = threading.Thread(target=monitor_thread_func, args=(controller,), daemon=True)
    monitor_thread.start()

    timer_thread = threading.Thread(target=timer_thread_func, args=(mission_timer,), daemon=True)
    timer_thread.start()

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
    lap_waypoints = load_lap_waypoints(os.path.join(package_share_dir, 'Waypoints')+"/coverage_waypoints.json")
    upload_mission(controller, home_pos, lap_waypoints, altitude=7)
    arm_drone(controller)
    takeoff_drone(controller, altitude=7)
    time.sleep(8)

    publisher_thread = threading.Thread(target=PubThread_Func, args=(controller,), daemon=True)
    publisher_thread.start()
    
    set_mode(controller, mode=3)
    time.sleep(2)
    
    mission_timer.start()
    print("\nMission started! Timer running...")
    start_mission(controller)

    try:
        while True:
            msg = controller.recv_match(type='MISSION_CURRENT', blocking=True, timeout=1)
            if msg and msg.seq == len(lap_waypoints) + 1:  
                mission_timer.stop()
                final_time = mission_timer.get_elapsed_time()
                minutes = int(final_time // 60)
                seconds = int(final_time % 60)
                print(f"\nMission Complete! Total time: {minutes:02d}:{seconds:02d}")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        mission_timer.stop()
        print("\nMission interrupted by user")
        final_time = mission_timer.get_elapsed_time()
        minutes = int(final_time // 60)
        seconds = int(final_time % 60)
        print(f"Final time before interruption: {minutes:02d}:{seconds:02d}")
    except Exception as e:
        mission_timer.stop()
        print(f"\nMission terminated due to error: {str(e)}")
        final_time = mission_timer.get_elapsed_time()
        minutes = int(final_time // 60)
        seconds = int(final_time % 60)
        print(f"Final time before error: {minutes:02d}:{seconds:02d}")

if __name__ == "__main__":
    main()
