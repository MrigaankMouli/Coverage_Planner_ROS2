import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Vector3
from pymavlink import mavutil
import time

SERIAL_PORT_CUBEPILOT = "udp:127.0.0.1:14550"

class CubePilotOdometryNode(Node):
    def __init__(self):
        super().__init__('cube_pilot_odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/gps', 10)
        self.get_logger().info(f"Attempting to connect with CubePilot on {SERIAL_PORT_CUBEPILOT}")
        
        try:
            self.CubePilot = mavutil.mavlink_connection(SERIAL_PORT_CUBEPILOT)
        except Exception:
            self.get_logger().error(f"CubePilot is not connected on {SERIAL_PORT_CUBEPILOT}")
            return
        
        self.CubePilot.wait_heartbeat()
        self.get_logger().info(f"Heartbeat from system (system {self.CubePilot.target_system} component {self.CubePilot.target_component})")
        self.get_logger().info("Connection to CubePilot is Successful")

        self.timer = self.create_timer(0.5, self.publish_odom)
        
        self.CubePilot.mav.request_data_stream_send(self.CubePilot.target_system, self.CubePilot.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 1, 1)

    def publish_odom(self):

        self.CubePilot.mav.command_long_send(
            self.CubePilot.target_system,
            self.CubePilot.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,
            0, 0, 0, 0,
            1, 1, 1
        )


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

def main(args=None):
    rclpy.init(args=args)
    cube_pilot_node = CubePilotOdometryNode()
    rclpy.spin(cube_pilot_node)
    cube_pilot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()