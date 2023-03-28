import sys
from pymavlink import mavutil
import time
import argparse



def change_mode(master, mode_id, target_sysid):
    master.mav.command_long_send(target_sysid, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result != 0:
        print(
            f"Error changing mode. Command Returned {msg.result}")
        print(f"{mavutil.mavlink.enums['MAV_RESULT'][msg.result].description}")
        return -1
    else:
        print(f"drone sysid {target_sysid} changed modes successfully")


def main(num_vehicles, mode):

    
    while True:
        master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
        master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" %
            (master.target_system, master.target_component))
        if master.target_system != 255:
            break


    # Check if mode is available
    if mode not in master.mode_mapping():
        print(f'Unknown mode : {mode}')
        print(f"available modes: {list(master.mode_mapping().keys())}")
        sys.exit(1)

    # Get mode ID
    mode_id = master.mode_mapping()[mode]

    for i in range(num_vehicles):
        print(f"sysid {i+1} changing modes")
        change_mode(master, mode_id, i+1)
        




if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Change flight mode')
    parser.add_argument('--mode', type=str, default='GUIDED', help='Flight mode')
    parser.add_argument('-i', type=int, default=1, help='Number of vehicles')


    args = parser.parse_args()

    print(args)
    main(args.i, args.mode)