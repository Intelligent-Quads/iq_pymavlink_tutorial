from multiprocessing import connection
from socket import timeout
import sys
from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
connection.wait_heartbeat()

# get type of autopilot
msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
autopilot_info = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].description

# ask for software version
connection.mav.send(mavutil.mavlink.MAVLink_autopilot_version_request_message(
    connection.target_system, connection.target_component))
msg = connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)

if msg:
    major = str(msg.flight_sw_version >> 24)
    sub = str(msg.flight_sw_version >> 16 & 0xFF)
    rev = str(msg.flight_sw_version >> 8 & 0xFF)

    fc_version = f"{major}.{sub}.{rev}"
    git_hash = ''.join(chr(i) for i in msg.flight_custom_version)
    print(autopilot_info)
    print(f"version is {fc_version} from git revision {git_hash}")
else:
    print("request timed out")
