# takeoff.py
import argparse
from pymavlink import mavutil
import time

def get_enum_value_by_name(enum_dict, name):
    for key, enum_entry in enum_dict.items():
        if enum_entry.name == name:
            return key
    raise ValueError("No enum entry with name: " + name)

def wait_until_position_aiding(mav_connection):
    flags = ['EKF_PRED_POS_HORIZ_REL', 'EKF_PRED_POS_HORIZ_REL']
    time_start = time.time()
    while not ekf_pos_aiding(mav_connection, flags) and time.time() - time_start < 120:
        print(time.time() - time_start)
        continue
    return

def ekf_pos_aiding(mav_connection, flags):
    msg = mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=3)
    if not msg:
        return 0
    print(f"from sysid {msg.get_srcSystem()} {msg}")
    ekf_flags = msg.flags

    for flag in flags:
        flag_val = get_enum_value_by_name(mavutil.mavlink.enums['EKF_STATUS_FLAGS'], flag)
        if ekf_flags & flag_val:
            continue
        else:
            return 0

    return 1



def takeoff(mav_connection, takeoff_altitude):
    mav_connection.wait_heartbeat()
    wait_until_position_aiding(mav_connection)
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mode_id = mav_connection.mode_mapping()["GUIDED"]


    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_altitude)

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    return takeoff_msg.result

def main():
    parser = argparse.ArgumentParser(description="A simple script to command a UAV to takeoff.")
    parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)

    args = parser.parse_args()

    mav_connection = mavutil.mavlink_connection('udpin:localhost:14551')
    takeoff(mav_connection, args.altitude)

if __name__ == "__main__":
    main()