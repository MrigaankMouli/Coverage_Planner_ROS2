from pymavlink import mavutil
import time

def set_speed(controller, speed, speed_type=1):

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
    controller = connect_to_vehicle("udpin:127.0.0.1:14550")

    if not controller:
        return

    print("Changing Climb Speed")
    if not set_speed(controller,speed=0.5,speed_type=2):
        return
    
    time.sleep(2)
    
    print("Changing Descent Speed")
    if not set_speed(controller,speed=0.5,speed_type=3):
        return

    time.sleep(2)

    if not set_mode(controller,4):
        return

    if not arm(controller):
        return

    time.sleep(5)

    if not takeoff(controller, altitude=3):
        return

    while True:
        msg = controller.recv_match(type=['GLOBAL_POSITION_INT'], blocking=True)
        relative_alt = msg.relative_alt / 1000.0
        print(f"Current Relative Altitude: {relative_alt} meters")
        if relative_alt >=2.5:
            print("Target Altitude Reached")
            break
    
    time.sleep(10)
 

if __name__ == "__main__":
    main()
