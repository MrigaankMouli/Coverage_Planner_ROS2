import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist, Vector3
from pymavlink import mavutil
import time
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

        self.timer = self.create_timer(0.1

 , self.publish_odom)

    def publish_odom(self):

        self.CubePilot.mav.request_data_stream_send(
            self.CubePilot.target_system,
            self.CubePilot.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  
            50,  
            1
        )

        msg = self.CubePilot.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        msg1 = self.CubePilot.recv_match(type='ATTITUDE', blocking=False)

        if msg and msg1:
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom_frame'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position = Point(
                x=msg.y, 
                y=msg.x, 
                z=-msg.z
            )

            r = R.from_euler('zyx', [-msg1.yaw, msg1.pitch, msg1.roll], degrees=False)
            quaternion = r.as_quat()
            odom.pose.pose.orientation.x = quaternion[0]
            odom.pose.pose.orientation.y = quaternion[1]
            odom.pose.pose.orientation.z = quaternion[2]
            odom.pose.pose.orientation.w = quaternion[3]

            odom.twist.twist.linear = Vector3(
                x=msg.vy, 
                y=msg.vx, 
                z=-msg.vz
            )
            odom.twist.twist.angular = Vector3(
                x=msg1.rollspeed, 
                y=msg1.pitchspeed, 
                z=-msg1.yawspeed
            )

            self.odom_pub.publish(odom)
            self.get_logger().info(f"Published Odometry: X={msg.x}, Y={msg.y}, Z={msg.z}")
        else:
            self.get_logger().warn("LOCAL_POSITION_NED/ATTITUDE message not available.")


        msg2 = self.CubePilot.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg2:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = 'gps_link'
            gps_msg.latitude = msg2.lat / 1e7
            gps_msg.longitude = msg2.lon / 1e7
            gps_msg.altitude = msg2.alt / 1000.0
            gps_msg.status.status = msg2.fix_type
            self.gps_pub.publish(gps_msg)
            self.get_logger().info(f"Published GPS: Latitude={gps_msg.latitude}, Longitude={gps_msg.longitude}, Altitude={gps_msg.altitude}")
        else:
            self.get_logger().warn("GPS_RAW_INT message not available.")

        msg3 = self.CubePilot.recv_match(type='SCALED_IMU2', blocking=False)
        if msg3 and msg1:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            imu_msg.linear_acceleration.x = msg3.xacc * float(0.00980665)  # Convert mG to m/s^2
            imu_msg.linear_acceleration.y = msg3.yacc * float(0.00980665)
            imu_msg.linear_acceleration.z = msg3.zacc * float(0.00980665)
            imu_msg.angular_velocity.x = (msg3.xgyro / float(1000))
            imu_msg.angular_velocity.y = (msg3.ygyro / float(1000))
            imu_msg.angular_velocity.z = (msg3.zgyro / float(1000))

            r = R.from_euler('zyx', [-msg1.yaw, msg1.pitch, msg1.roll], degrees=False)
            quaternion = r.as_quat()
            imu_msg.orientation.x = quaternion[0]
            imu_msg.orientation.y = quaternion[1]
            imu_msg.orientation.z = quaternion[2]
            imu_msg.orientation.w = quaternion[3]

            self.imu_pub.publish(imu_msg)
            self.get_logger().info("Published IMU data.")
        else:
            self.get_logger().warn("SCALED_IMU2 or ATTITUDE message not available.")


def main(args=None):
    controller = mavutil.mavlink_connection("/dev/ttyACM0")
    controller.wait_heartbeat()
    print("Connection to CubeOrange Successful")

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1,
        0, 0, 0, 0, 0, 0)

    rclpy.init(args=args)
    cube_pilot_node = CubePilotOdometryNode(controller)
    rclpy.spin(cube_pilot_node)
    cube_pilot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()