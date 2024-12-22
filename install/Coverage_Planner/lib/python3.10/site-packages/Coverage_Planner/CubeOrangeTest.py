from pymavlink import mavutil
import rclpy

SERIAL_PORT_CUBEPILOT = "udpin:127.0.0.1:14550"

def main():
    print(f"Attempting to Connect with CubePilot on {SERIAL_PORT_CUBEPILOT}")
    CubePilot = mavutil.mavlink_connection(SERIAL_PORT_CUBEPILOT)
    CubePilot.wait_heartbeat()
    print(f"Heartbeat from system (system {CubePilot.target_system} component {CubePilot.target_component})")
    print("Connection to CubePilot is Sucsessfull")

    CubePilot.mav.request_data_stream_send(
        CubePilot.target_system,
        CubePilot.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  
            10000,  
            1
            )

    msg3 = CubePilot.recv_match(type='RAW_IMU', blocking=True)
    print(msg3)