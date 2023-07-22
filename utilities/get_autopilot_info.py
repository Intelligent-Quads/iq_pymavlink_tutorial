from pymavlink import mavutil
import time
import argparse

def get_autopilot_info(connection, sysid=1):
    autopilot_info = {"autopilot": "", "type": "", "version": ""}
    heartbeat_sysid = 0
    while heartbeat_sysid != sysid:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        heartbeat_sysid = msg.get_srcSystem()
    print(msg)
    autopilot = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name.replace("MAV_AUTOPILOT_", "").lower()
    autopilot_info["autopilot"] = autopilot
    autopilot_info["type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name.replace("MAV_TYPE_", "").lower()
    
    if autopilot_info["autopilot"] == "ardupilotmega":
        connection.mav.send(mavutil.mavlink.MAVLink_autopilot_version_request_message(
            connection.target_system, connection.target_component))
        msg = connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
        if msg:
            major = str(msg.flight_sw_version >> 24)
            sub = str(msg.flight_sw_version >> 16 & 0xFF)
            rev = str(msg.flight_sw_version >> 8 & 0xFF)
            fc_version = f"{major}.{sub}.{rev}"
            git_hash = ''.join(chr(i) for i in msg.flight_custom_version)
            autopilot_info["version"] = fc_version
    return autopilot_info

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Get autopilot info')
    parser.add_argument('--sysid', type=int, default=1, help='System ID to search for (default: 1)')
    args = parser.parse_args()

    connection = mavutil.mavlink_connection('udpin:localhost:14551')
    autopilot_info = get_autopilot_info(connection, args.sysid)
    print(autopilot_info)
