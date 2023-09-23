from pymavlink import mavutil
import argparse
from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info

def check_alt_reached(connection, alt):
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
    if abs(msg.z - (-alt)) < 1:
        return 1
    else:
        return 0


def check_reached(connection):
    msg = connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    print(msg)
    if (msg.wp_dist < 1):
        return 1
    else:
        return 0


def set_new_wp_ned(the_connection, wp_ned):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component,
                                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), wp_ned[0], wp_ned[1], wp_ned[2], 0, 0, 0, 0, 0, 0, 1.5, 0))
    while 1:
        msg = the_connection.recv_match(
            type='NAV_CONTROLLER_OUTPUT', blocking=True)
        if msg.wp_dist > 0:
            break


def square(the_connection, args):

    mode_id = the_connection.mode_mapping()["GUIDED"]
    # Change mode to guided (Ardupilot) or takeoff (PX4)
    the_connection.mav.command_long_send(args.sysid, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")



    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    alt = 10
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 25, 0, 0, alt)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    while 1:
        if check_alt_reached(the_connection, alt):
            print("takeoff reached")
            break


    wp_arr = []
    wp_arr.append([10, 0, -10])
    wp_arr.append([10, 10, -10])
    wp_arr.append([0, 10, -10])
    wp_arr.append([0, 0, -10])

    wp_num = 0
    for wp in wp_arr:
        set_new_wp_ned(the_connection, wp)
        while 1:
            if check_reached(the_connection):
                break
        wp_num = wp_num + 1

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

def main():
    parser = argparse.ArgumentParser(description="A simple script to command a UAV to takeoff.")
    # parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)
    parser.add_argument("--sysid", type=int, help="System ID of the UAV to command.", default=1)


    args = parser.parse_args()
    the_connection = connect_to_sysid('udpin:localhost:14551', args.sysid)
    wait_until_position_aiding(the_connection)
    square(the_connection, args)
    

if __name__ == "__main__":
    main()