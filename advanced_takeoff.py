
from pymavlink import mavutil
import time
import sys


def get_version_info(con):
    msg = con.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
    autopilot_info = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].description
    print(mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].description)
    print(mavutil.mavlink.enums['MAV_TYPE'][msg.type].description)


def connect_to_sysid(connection_str, sysid, timeout=3):

    time_start = time.time()
    while time.time() - time_start < 3:
        the_connection = mavutil.mavlink_connection(connection_str)
        the_connection.wait_heartbeat()
        print(
            f"Heartbeat from system system {the_connection.target_system} component {the_connection.target_component}")
        if the_connection.target_system == sysid:
            print(f"Now connected to SYSID {sysid}")
            return the_connection

    print(f"unable to connect to sysid {sysid}")
    sys.exit(-1)


def ekf_pos_aiding(flags):
    msg = con.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=3)
    ekf_flags = msg.flags

    for flag in flags:

        if ekf_flags & mavutil.mavlink.enums['EKF_STATUS_FLAGS'][flag]:
            continue
        else:
            return 0

    return 1


if __name__ == "__main__":
    con = connect_to_sysid('udpin:localhost:14551', 2)
    # Start a connection listening to a UDP port

    get_version_info(con)
    print(ekf_pos_aiding(["EKF_PRED_POS_HORIZ_REL", "EKF_PRED_POS_HORIZ_ABS"]))
