from pymavlink import mavutil, mavwp
import json
import argparse
from typing import Dict, Union


def read_qgc_mission(mission_file: str) -> Dict:
    """
    Reads a mission file from QGroundControl.

    Args:
        mission_file (str): The path to the mission file.

    Returns:
        dict: A dictionary containing the mission details.
    """
    try:
        with open(mission_file, 'r') as file:
            mission = json.load(file)
    except FileNotFoundError:
        print(f"Mission file {mission_file} not found.")
        return {}

    return mission


def upload_qgc_mission(mission_file: str, master: mavutil.mavlink_connection) -> Union[bool, None]:
    """
    Uploads a mission to UAV.

    Args:
        mission_file (str): The path to the mission file.
        master (mavutil.mavlink_connection): The MAVLink connection to use.

    Returns:
        bool: True if the mission is successfully uploaded, False otherwise.
    """
    mission = read_qgc_mission(mission_file)
    if not mission:
        return None

    # Create MAVLink waypoint loader
    wploader = mavwp.MAVWPLoader()

    seq = 0  # Waypoint sequence begins at 0
    # Ardupilot expects the home position to be the first waypoint
    # Adding home position waypoint
    wploader.add(mavutil.mavlink.MAVLink_mission_item_message(
        0, 0, seq, 0, 16, 0, 0, 0, 0, 0, 0,
        float(mission["mission"]["plannedHomePosition"][0]), 
        float(mission["mission"]["plannedHomePosition"][1]), 
        float(mission["mission"]["plannedHomePosition"][2])
    ))

    seq += 1  # Increase waypoint sequence

    # Add all waypoints from mission to MAVLink waypoint loader
    for wp in mission["mission"]["items"]:
        print(wp)
        wploader.add(mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, seq, wp["frame"], wp["command"], 0, int(wp["autoContinue"]), 
            float(wp["params"][0]), float(wp["params"][1]), float(wp["params"][2]), 
            float(wp["params"][3]), float(wp["params"][4]), float(wp["params"][5]), 
            float(wp["params"][6])
        ))
        seq += 1  # Increase waypoint sequence for the next waypoint
    
    # Clear any existing mission from vehicle
    print('Clear mission')
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    # Confirm mission cleared
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
    if ack.type != 0:
        print(f'Error clearing mission: {ack.type}')
        return False

    # Send waypoint count to the UAV
    master.waypoint_count_send(wploader.count())

    # Upload waypoints to the UAV
    for i in range(wploader.count()):
        # Wait for MAVLink mission request message
        msg = master.recv_match(type=['MISSION_REQUEST'], blocking=True)
        print(msg)
        # Print waypoint data
        print(wploader.wp(msg.seq))
        # Send waypoint to the UAV
        master.mav.send(wploader.wp(msg.seq))
        print(f'Sending waypoint {msg.seq}')

    # Confirm mission upload
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
    if ack.type != 0:
        print(f'Error uploading mission: {ack.type}')
        return False

    return True


if __name__ == '__main__':
    # Parser for command-line arguments
    parser = argparse.ArgumentParser(description='Upload a mission created by QGroundControl to a vehicle.')

    # Add mission_file argument
    parser.add_argument('--mission_file', type=str, default='wps/CMAC_square.plan',
                        help='Path to the mission file.')

    # Add address argument
    parser.add_argument('--address', type=str, default='udpin:localhost:14551',
                        help='The address and port to connect to.')
    
    # Parse the arguments
    args = parser.parse_args()

    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection(args.address)

    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()

    print(f"Heartbeat from system (system {the_connection.target_system} component {the_connection.target_component})")

    # Upload the mission to the UAV
    upload_qgc_mission(args.mission_file, the_connection)
