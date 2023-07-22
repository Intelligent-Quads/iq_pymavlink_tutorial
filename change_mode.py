import sys
from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
import argparse

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
