import sys
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

# Choose a mode
mode = 'FOLLOW'

# Check if mode is available
if mode not in master.mode_mapping():
    print(f'Unknown mode : {mode}')
    print(f"available modes: {list(master.mode_mapping().keys())}")
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]


master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                             0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)


time_start = time.time()
while time.time() - time_start < 3:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(ack_msg, mavutil.mavlink.MAV_CMD_DO_SET_MODE)
    if ack_msg:
        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg.command != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue
        else:
            # Print the ACK result !
            print(mavutil.mavlink.enums['MAV_RESULT']
                  [ack_msg.result].description)
            break
