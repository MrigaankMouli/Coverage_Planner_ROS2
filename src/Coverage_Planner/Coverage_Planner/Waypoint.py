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
    print("Drone armed.")
    time.sleep(5)

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

def send_local_position(controller, north, east, down=-3):
    controller.mav.set_position_target_local_ned_send(
        0,
        controller.target_system, controller.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        north, east, down,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )


def get_lat_lon(controller):
    msg = controller.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    latitude = msg.lat
    longitude = msg.lon
    return latitude / 1e7, longitude / 1e7


def navigate_waypoints(controller, waypoints, altitude=-3):
    coordinates = []
    for north, east in waypoints:
        send_local_position(controller, north, east, altitude)
        print(f"Moving to waypoint: North {north}, East {east}, Down {altitude}")

        while True:
            msg = controller.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current_north = msg.x
            current_east = msg.y
            if abs(current_north - north) < 0.2 and abs(current_east - east) < 0.2:
                print("Waypoint reached")
                lat, lon = get_lat_lon(controller)
                print(f"Current Lat, Lon: {lat}, {lon}")
                coordinates.append({"lat": lat, "lon": lon})
                break
            time.sleep(1)

    return coordinates


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


def save_coordinates_to_file(coordinates, filename='coverage_boundary.json'):
    with open(filename, 'w') as json_file:
        json.dump(coordinates, json_file)
    print(f"Coordinates saved to {filename}")

SERIAL_PORT_CUBEPILOT = "udpin:127.0.0.1:14550"

class CubePilotOdometryNode(Node):
    """
    Node to Publish the Odometry, IMU and GPS data from the GPS and the CubeOrange IMU as ROS2 messages

    """

    def __init__(self,controller):
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
            10,  
            1
        )
        msg = self.CubePilot.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        msg1 = self.CubePilot.recv_match(type='ATTITUDE', blocking=False)
        
        if msg:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose.position = Point(x=msg.x, y=msg.y, z=msg.z)
            odom.twist.twist.linear = Vector3()
            odom.twist.twist.linear.x = msg.vx
            odom.twist.twist.linear.y = msg.vy
            odom.twist.twist.linear.z = msg.vz
            
            if msg1:
                odom.twist.twist.angular = Vector3()
                odom.twist.twist.angular.x = msg1.roll
                odom.twist.twist.angular.y = msg1.pitch
                odom.twist.twist.angular.z = msg1.yaw
            else:
                self.get_logger().warn("ATTITUDE message not available, skipping angular velocities.")
            
            self.odom_pub.publish(odom)
            self.get_logger().info(f"Position (NED): X: {msg.x}, Y: {msg.y}, Z: {msg.z}")
            self.get_logger().info(f"Velocity: Vx: {msg.vx}, Vy: {msg.vy}, Vz: {msg.vz}")
    
        msg2 = self.CubePilot.recv_match(type="GPS_RAW_INT", blocking=False)
        
        if msg2:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps'
            gps_msg.latitude = msg2.lat / 1e7
            gps_msg.longitude = msg2.lon / 1e7
            gps_msg.altitude = msg2.alt / 1000
            gps_msg.status.status = msg2.fix_type   

            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Published GPS data: Latitude={gps_msg.latitude}, Longitude={gps_msg.longitude}, Altitude={gps_msg.altitude}")
        else:
            self.get_logger().warn("GPS Message not available, skipping GPS coordinates ")    

        msg3 = self.CubePilot.recv_match(type='HIGHRES_IMU',blocking=False)

        if msg3:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id ='imu'
            imu_msg.linear_acceleration.x = msg.xacc
            imu_msg.linear_acceleration.y = msg.yacc
            imu_msg.linear_acceleration.z = msg.zacc

            imu_msg.angular_velocity.x = msg.xgyro
            imu_msg.angular_velocity.y = msg.ygyro
            imu_msg.angular_velocity.z = msg.zgyro

            r = R.from_euler('xyz', [msg1.roll, msg1.pitch, msg1.yaw], degrees=False)
            quaternion = r.as_quat()

            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]
            
            self.imu_pub.publish(imu_msg)
            self.get_logger().info("Published IMU data with quaternion.")
        else:
            self.get_logger().warn("IMU Message not Availible.")


def main():
    print("Initializing connection...")
    controller = mavutil.mavlink_connection(SERIAL_PORT_CUBEPILOT)
    controller.wait_heartbeat()
    print("Connection established.")

    set_mode(controller, 4)
    arm_drone(controller)
    takeoff_drone(controller, altitude=7)

    Odom_Pub_thread = threading.Thread(target=OdomThread_Func,args=(controller,),daemon=True)
    Odom_Pub_thread.start()

    square_points = [
        (10, 0),
        (16, 2),
        (10, 3),
        (0, 0)
    ]

    coordinates = navigate_waypoints(controller, square_points, altitude=-7)
    save_coordinates_to_file(coordinates)

    land_vehicle(controller)

    os._exit(0)

def OdomThread_Func(controller):
    """
    Thread to start the Odometry, IMU and GPS data Publisher

    """

    controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,
            0, 0, 0, 0,
            1, 1, 1
        )


    rclpy.init(args=None)
    cube_pilot_node = CubePilotOdometryNode(controller)

    rclpy.spin(cube_pilot_node)

if __name__ == "__main__":
    main()
