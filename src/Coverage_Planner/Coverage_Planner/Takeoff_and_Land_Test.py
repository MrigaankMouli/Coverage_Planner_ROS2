from pymavlink import mavutil
import time
import threading


def monitor_thread_func(controller):
    
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
                os._exit(o)
            else:
                print("Throttle value too low. Control remains with the script")

        time.sleep(0.2)
    
def set_speed(controller, speed, speed_type):

    try:
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
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Speed set to {speed} m/s successfully.")
        else:
            print(f"Failed to set speed: {msg.result}")
    except Exception as e:
        print(f"Error setting speed: {e}")

def connect_to_vehicle(connection_string):
    print("Connecting to vehicle...")
    try:
        controller = mavutil.mavlink_connection(connection_string)
        controller.wait_heartbeat()
        print("Connected to vehicle")
        return controller
    except Exception as e:
        print(f"Connection Failed. Error: {e}")
        return None

def set_mode(controller, mode):
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

def arm(controller):
    print("Arming vehicle...")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        print("Vehicle armed successfully")
        return True
    except Exception as e:
        print(f"Failed to arm vehicle. Error: {e}")
        return False

def disarm(controller):
    print("Disarming vehicle...")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        print("Vehicle disarmed successfully")
        return True
    except Exception as e:
        print(f"Failed to disarm vehicle. Error: {e}")
        return False

def takeoff(controller, altitude):
    print(f"Initiating takeoff to {altitude} meters...")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0, 0, 0, 0, altitude
        )
        print("Takeoff command sent")
        return True
    except Exception as e:
        print(f"Failed to send takeoff command. Error: {e}")
        return False

def land(controller):
    print("Initiating landing...")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        print("Landing command sent")
        return True
    except Exception as e:
            print(f"Failed to send landing command. Error: {e}")
            return False

def main():

    controller = connect_to_vehicle("/dev/ttyACM0")

    if not controller:
        return

    monitor_thread = threading.Thread(target=monitor_thread_func,args=(controller,),daemon=True)
    monitor_thread.start()

    if not set_mode(controller,4):
        return

    if not arm(controller):
        return
    
    time.sleep(5)

    if not takeoff(controller, altitude=7):
        return
    
    print("Hovering for fifteen seconds")
    time.sleep(15)

    if not land(controller):
        return


if __name__ == "__main__":
    main()
