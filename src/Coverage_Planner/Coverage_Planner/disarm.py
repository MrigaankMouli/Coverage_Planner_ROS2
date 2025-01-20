from pymavlink import mavutil
import time

def disarm(controller, max_retries=5):
    print("Sending DISARM command...")

    retries = 0
    while retries < max_retries:
        controller.mav.command_long_send(
            controller.target_system,
            controller.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0, 0, 0, 0, 0
        )

        command_ack = controller.recv_match(type="COMMAND_ACK", blocking=True, timeout=10)

        if command_ack is None:
            print("Timeout waiting for COMMAND_ACK for DISARM command. Retrying...")
        elif command_ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if command_ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("DISARM command accepted.")
                return True
            else:
                print(f"DISARM command rejected. Retrying...")
        
        retries += 1
        time.sleep(0.1)

    print("DISARM command failed after maximum retries.")
    return False


def main():
    print("Initializing connection...")
    controller = mavutil.mavlink_connection("/dev/ttyACM0")
    controller.wait_heartbeat()
    print("Connection Established")

    command_ack = disarm(controller)

    if command_ack:
        print("Disarm Command Accepted")
    else:
        print("Disarm Command Rejected after 5 retries")
