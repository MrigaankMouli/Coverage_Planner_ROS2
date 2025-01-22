from pymavlink import mavutil
import time

def land(controller, max_retries=5):
    print("Sending LAND command...")

    retries = 0
    while retries < max_retries:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0, 0, 0, 0, 0
        )

        command_ack = controller.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)

        if command_ack is None:
            print("Timeout waiting for COMMAND_ACK for LAND command. Retrying...")
        elif command_ack.command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            if command_ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("LAND command accepted.")
                return True
            else:
                print(f"LAND command rejected. Retrying...")
        retries += 1
        time.sleep(0.1)

    print("LAND command failed after maximum retries.")
    return False


def main():
    print("Initializing connection...")
    controller = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    controller.wait_heartbeat()
    print("Connection Established")

    command_ack = land(controller)

    if command_ack:
        print("Land Command Accepted")
    else:
        print("Land Command Rejected after 5 retries")
    


