
from pymavlink import mavutil
import time
import sys
import argparse


def recv_match_sysid(sysid, con, type, timeout=3):
    """recv_match_sysid receives a mavlink message from a specific sysid

    Args:
        sysid (int): sysid to listen for
        con (mavutil): mavutil connection object
    """    
    time_start = time.time()
    while time.time() - time_start < timeout:
        msg = con.recv_match(type=type, blocking=True)
        if msg.get_srcSystem() == sysid:
            return msg
    return {}

def get_version_info(con):
    """get_version_info gets the version info from the autopilot

    Args:
        con (mavutil): mavutil connection object
    """    
    msg = recv_match_sysid(con.target_system, con, 'HEARTBEAT', timeout=3)
    print(msg)
    print(msg.get_srcSystem())
    print(mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].description)
    print(mavutil.mavlink.enums['MAV_TYPE'][msg.type].description)


def connect_to_sysid(connection_str : str, sysid : int, timeout: float = 3) -> any:
    """connect_to_sysid connects to a mavlink stream with a specific sysid

    Args:
        connection_str (str, optional): _description_. Defaults to 3)->mavutil.mavlink_connection(.

    Returns:
        _type_: _description_
    """    
    time_start = time.time()
    while time.time() - time_start < timeout:
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
        print(flag)
        if ekf_flags & mavutil.mavlink.enums['EKF_STATUS_FLAGS'][flag]:
            continue
        else:
            return 0

    return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sysid", type=int, default=1)

    args = parser.parse_args()

    con = connect_to_sysid('udpin:localhost:14551', args.sysid)
    # Start a connection listening to a UDP port

    get_version_info(con)
    print(ekf_pos_aiding(["EKF_PRED_POS_HORIZ_REL", "EKF_PRED_POS_HORIZ_ABS"]))
