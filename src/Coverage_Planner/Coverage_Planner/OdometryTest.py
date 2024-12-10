from pymavlink import mavutil
import rclpy
import time

def main():
    controller = mavutil.mavlink_connection("/dev/ttyACM0")
    controller.wait_heartbeat()

    print("Connected to vehicle")

    controller.mav.command_long_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1,
        0, 0, 0, 0,
        1, 1, 1
    )


    controller.mav.request_data_stream_send(
        controller.target_system,
        controller.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,  
        10,  
        1
    )
     
    while True:

        msg = controller.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            print("Global Position:", msg)
        else:
            print("No Position Data")

        msg = controller.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            print("Local Position:", msg)
        else:
            print("No Local Position Data")

        msg = controller.recv_match(type='ATTITUDE', blocking=False)
        if msg:
            print("Attitude:", msg)
        else:
            print("No Attitude Data")

        time.sleep(1)

if __name__ == "__main__":
    main()
