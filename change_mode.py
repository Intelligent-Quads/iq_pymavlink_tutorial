import sys
from pymavlink import mavutil
import argparse
import time

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

def change_mode(master, mode):

    # Check if mode is available
    if mode not in master.mode_mapping():
        print(f'Unknown mode : {mode}')
        print(f"available modes: {list(master.mode_mapping().keys())}")
        sys.exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]


    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(ack_msg)
    return ack_msg.result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Change mode of the drone')
    parser.add_argument('--mode', type=str, default='GUIDED', help='Mode to change to')
    parser.add_argument("--sysid", type=int, default=1)
    args = parser.parse_args()

    master = connect_to_sysid('udpin:localhost:14551', args.sysid)
    
    # wait for the heartbeat msg to find the system ID
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

    change_mode(master, args.mode)
