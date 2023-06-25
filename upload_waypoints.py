from pymavlink import mavutil
from pymavlink import mavwp
import json
import argparse

# Function to read mission file from QGroundControl
def read_qgc_mission(mission_file):
    # Load mission from file
    mission = json.load(open(mission_file))
    return mission

# Function to upload mission to UAV
def upload_qgc_mission(mission, master):
    # Create MAVLink waypoint loader
    wploader = mavwp.MAVWPLoader()

    # Ardupilot expects the home position to be the first waypoint
    seq = 0  # Waypoint sequence begins at 0
    wploader.add(mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, seq, 0, 16, 0, 0, 0, 0, 0, 0,
            # Adding home position waypoint
            float(mission["mission"]["plannedHomePosition"][0]), 
            float(mission["mission"]["plannedHomePosition"][1]), 
            float(mission["mission"]["plannedHomePosition"][2])
        )
    )

    # Increase waypoint sequence
    seq += 1

    # Add all waypoints from mission to MAVLink waypoint loader
    for wp in mission["mission"]["items"]:
        print(wp)

        wploader.add(mavutil.mavlink.MAVLink_mission_item_message(
            0, 0, seq, wp["frame"], wp["command"], 0, int(wp["autoContinue"]), 
            float(wp["params"][0]), float(wp["params"][1]), float(wp["params"][2]), 
            float(wp["params"][3]), float(wp["params"][4]), float(wp["params"][5]), 
            float(wp["params"][6])
        ))
        # Increase waypoint sequence for the next waypoint
        seq += 1
    
    # Clear any existing mission from vehicle
    print('Clear mission')
    master.mav.mission_clear_all_send(master.target_system, master.target_component)

    # Confirm mission cleared
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
    if ack.type != 0:
        print('Error clearing mission: {0}'.format(ack.type))
        return False

    # Send waypoint count to the UAV
    master.waypoint_count_send(wploader.count())

    # Upload waypoints to the UAV
    for i in range(wploader.count()):
        # Wait for MAVLink mission request message
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        # Print waypoint data
        print(wploader.wp(msg.seq))
        # Send waypoint to the UAV
        master.mav.send(wploader.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))

    # Confirm mission upload
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=3)
    print(ack)
    if ack.type != 0:
        print('Error uploading mission: {0}'.format(ack.type))
        return False
    return True

def main(mission_file):
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()

    print("Heartbeat from system (system %u component %u)" % 
          (the_connection.target_system, the_connection.target_component))

    # Upload the mission to the UAV
    upload_qgc_mission(read_qgc_mission(mission_file), the_connection)

if __name__ == '__main__':
    # Parser for command-line arguments
    parser = argparse.ArgumentParser(description='Upload a mission created by QGroundControl to a vehicle.')

    # Add mission_file argument
    parser.add_argument('--mission_file', type=str, default='wps/CMAC_square.plan',
                        help='Path to the mission file.')
    
    # Parse the arguments
    args = parser.parse_args()
    # Run the main function with the specified arguments
    main(args.mission_file)
