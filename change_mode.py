import sys
from pymavlink import mavutil
from utilities.connect_to_sysid import connect_to_sysid
import argparse

main_mode_mapping_px4 = {
    'MANUAL': 0,
    'ALTCTL': 1,
    'POSCTL': 2,
    'AUTO': 3,
    'ACRO': 4,
    'OFFBOARD': 5,
    'STABILIZED': 6,
    'RATTITUDE': 7,
}

sub_mode_mapping_px4 = {
    'READY': 0,
    'TAKEOFF': 1,
    'HOLD': 2,  # LOITER in MAVLink
    'MISSION': 3,
    'RETURN_TO_LAUNCH': 4,
    'LAND': 5,
    'FOLLOW_ME': 6,
}


def change_mode(master, mode, autopilot='ardupilot', sub_mode='NONE'):

    if autopilot == 'ardupilot':
        # Check if mode is available
        if mode not in master.mode_mapping():
            print(f'Unknown mode : {mode}')
            print(f"available modes: {list(master.mode_mapping().keys())}")
            raise Exception('Unknown mode')
        
        # Get mode ID
        mode_id = master.mode_mapping()[mode]
        sub_mode = 0
    elif autopilot == 'px4':
        # Get mode ID
        mode_id = main_mode_mapping_px4[mode]
        sub_mode = sub_mode_mapping_px4[sub_mode]


    master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, sub_mode, 0, 0, 0, 0)
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

    # change_mode(master, args.mode)
    print(change_mode(master, "AUTO", "px4", "READY"))
