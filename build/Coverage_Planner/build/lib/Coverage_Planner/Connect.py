from pymavlink import mavutil
import time


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

def arm(controller):
    print("Arming... ")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0, 0, 0, 0, 0, 0
        )
        print("Vehicle Armed Succesfully")
        return True
    except Exception as e:
        print(f"Failed to Arm. Error:{e}")
        return False


def disarm(controller):
    print("Disarming... ")
    try:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0, 0, 0, 0, 0, 0
        )
        print("Vehicle Disarmed Succesfully")
        return True
    except Exception as e:
        print(f"Failed to Disarm. Error:{e}")
        return False


def main():
    controller = mavutil.mavlink_connection("/dev/ttyACM0")
    controller.wait_heartbeat()
    print("Connected to vehicle")

    set_mode(controller,4)
    time.sleep(2)
    arm(controller)

    time.sleep(4)

    disarm(controller)



if __name__ == "__main__":
    main()