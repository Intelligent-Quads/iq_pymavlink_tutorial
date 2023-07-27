from pymavlink import mavutil
import time
import argparse
import os
import sys
utilities_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(utilities_path)
from connect_to_sysid import connect_to_sysid

from pymavlink import mavutil
from connect_to_sysid import connect_to_sysid


def get_autopilot_info(connection, sysid=1):
    """
    Get the autopilot information for the MAVLink connection.

    Args:
        connection (mavutil.mavlink_connection): The MAVLink connection.
        sysid (int, optional): System ID to search for. Defaults to 1.

    Returns:
        dict: A dictionary containing autopilot info. 
              Includes "autopilot", "type", and "version".
    """
    # Initialize dictionary to hold autopilot info
    autopilot_info = {"autopilot": "", "type": "", "version": ""}
    
    # Receive 'HEARTBEAT' message from MAVLink connection
    msg = wait_for_heartbeat(connection, sysid)
    # If no message received, return empty autopilot_info
    if not msg:
        return autopilot_info

    # Get autopilot type and MAV type from message and add to autopilot_info
    autopilot = mavutil.mavlink.enums['MAV_AUTOPILOT'][msg.autopilot].name.replace("MAV_AUTOPILOT_", "").lower()
    autopilot_info["autopilot"] = autopilot
    autopilot_info["type"] = mavutil.mavlink.enums['MAV_TYPE'][msg.type].name.replace("MAV_TYPE_", "").lower()

    # If autopilot type is ArduPilot Mega, request autopilot version and add to autopilot_info. I don't think this is implemented for PX4.
    if autopilot_info["autopilot"] == "ardupilotmega":
        msg = request_autopilot_version(connection)
        if msg:
            autopilot_info["version"] = get_fc_version_from_msg(msg)

    # Return the autopilot information
    return autopilot_info

def wait_for_heartbeat(connection, sysid, timeout=3):
    """
    Waits for a HEARTBEAT message from the given sysid
    
    Arguments:
    connection -- pymavlink connection object
    sysid      -- system id to search for
    timeout    -- time to wait for the message in seconds
    
    Returns:
    The HEARTBEAT message or None if timeout is reached
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=timeout)
        if msg.get_srcSystem() == sysid:
            return msg
    return None

def request_autopilot_version(connection):
    """Request and return an AUTOPILOT_VERSION message from a mavlink connection"""
    connection.mav.send(mavutil.mavlink.MAVLink_autopilot_version_request_message(
        connection.target_system, connection.target_component))
    return connection.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)


def get_fc_version_from_msg(msg):
    """Extract and return the flight controller version from an AUTOPILOT_VERSION message"""
    major = str(msg.flight_sw_version >> 24)
    sub = str(msg.flight_sw_version >> 16 & 0xFF)
    rev = str(msg.flight_sw_version >> 8 & 0xFF)
    return f"{major}.{sub}.{rev}"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Get autopilot info')
    parser.add_argument('--sysid', type=int, default=1, help='System ID to search for (default: 1)')
    parser.add_argument('--timeout', type=int, default=120, help='Maximum wait time for position aiding (default: 120 seconds)')
    parser.add_argument('--connection_str', type=str, default='udpin:localhost:14551', help='Connection string (default: udpin:localhost:14551)')
    args = parser.parse_args()

    # Connect to specified sysid
    connection = connect_to_sysid(args.connection_str, args.sysid)

    # Get autopilot information
    autopilot_info = get_autopilot_info(connection, args.sysid)
    print(autopilot_info)