import sys
from pymavlink import mavutil
from pymavlink import mavwp
import time
import argparse


def cmd_set_home(home_location, altitude, master):
    print('--- ', master.target_system, ',', master.target_component)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1, # set position
        0, # param1
        0, # param2
        0, # param3
        0, # param4
        home_location[0], # lat
        home_location[1], # lon
        altitude) 

    msg = master.recv_match(type = ['COMMAND_ACK'],blocking = True)
    print(msg)
    print('Set home location: {0} {1}'.format(home_location[0],home_location[1]))
    time.sleep(1)



# This script is used to upload a mavlink mission to a drone.

def upload_mission(mission_file, master):
    
    wp = mavwp.MAVWPLoader()

    # Read mission from file
    missionlist = []
    with open(mission_file) as f:
        for i, line in enumerate(f):
            if i == 0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray = line.split('\t')
                ln_index = int(linearray[0])
                ln_currentwp = int(linearray[1])
                ln_frame = int(linearray[2])
                ln_command = int(linearray[3])
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_param5 = float(linearray[8])
                ln_param6 = float(linearray[9])
                ln_param7 = float(linearray[10])
                ln_autocontinue = int(linearray[11].strip())
                p = mavutil.mavlink.MAVLink_mission_item_message(
                    0, 0, ln_index, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3,
                    ln_param4, ln_param5, ln_param6, ln_param7)
                wp.add(p)

    # Clear existing mission from vehicle
    print('Clear mission')
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component)

    # Confirm mission cleared
    while True:
        ack = master.recv_match(type='MISSION_ACK', blocking=True)
        if ack.type == 0:
            break
    
    master.waypoint_count_send(wp.count())
    for i in range(wp.count()):
        msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)
        print(msg)
        master.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))
    
    # Confirm mission uploaded
    while True:
        ack = master.recv_match(type='MISSION_ACK', blocking=True)
        if ack.type == 0:
            break

    print('Mission uploaded')
    

def main(mission_file, num_vehicles):

    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))

    for i in range(num_vehicles):
        upload_mission(mission_file, the_connection)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Upload a mission to a drone.')
    parser.add_argument('--mission_file', type=str, default='mission.txt',
                        help='Path to the mission file.')
    parser.add_argument('--num_vehicles', type=int, default=1,
                        help='Number of vehicles to upload the mission to.')
    args = parser.parse_args()
    main(args.mission_file, args.num_vehicles)