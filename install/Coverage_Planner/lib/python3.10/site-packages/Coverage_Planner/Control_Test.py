from pymavlink import mavutil
import threading
import time
import os

def monitor_thread_func(controller):

    print("Thread running")
    
    while True:
        msg = controller.recv_match(type=["RC_CHANNELS","HEARTBEAT"],blocking = True)
        if not msg:
            continue

        if msg.get_type() == 'HEARTBEAT':
            mode = mavutil.mode_string_v10(msg)
            
            if mode == "POSHOLD":
                print("Mode set to POSHOLD. Checking Throttle Value...")

                rc_msg = controller.recv_match(type = "RC_CHANNELS",blocking = True)
                if rc_msg and rc_msg.chan3_raw>1200:
                    print("Pilot taking over control. Exiting Script.")
                    os._exit(0)
                else:
                    print("Throttle value too low. Control remains with the script")
        
        time.sleep(0.1)


def main():
    controller = mavutil.mavlink_connection("/dev/ttyACM0")
    controller.wait_heartbeat()
    print("Connected to vehicle")

    monitor_thread = threading.Thread(target=monitor_thread_func,args=(controller,),daemon=True)
    monitor_thread.start()

    controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,
            0, 0, 0, 0, 0
        )

    while True:
        print("Control still with Script...")
        time.sleep(2)


if __name__ == '__main__':
    main()


