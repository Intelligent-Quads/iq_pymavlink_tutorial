# takeoff.py
import argparse
from pymavlink import mavutil

def takeoff(mav_connection, takeoff_altitude):
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(arm_msg)

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_altitude)

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(takeoff_msg)

    return takeoff_msg.result

def main():
    parser = argparse.ArgumentParser(description="A simple script to command a UAV to takeoff.")
    parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)

    args = parser.parse_args()

    mav_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    takeoff(mav_connection, args.altitude)

if __name__ == "__main__":
    main()
