import argparse
from pymavlink import mavutil

def land(the_connection):
    # Send a command to land
    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait for the acknowledgment
    ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(ack)

    # Print the result
    print(mavutil.mavlink.enums['MAV_RESULT'][ack.result].description)

    return ack.result

def main(args):
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection(args.connection_string)

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()

    # Call the land function
    land(the_connection)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--connection_string', default='udpin:localhost:14551', help='Connection string for mavlink')
    args = parser.parse_args()

    main(args)
