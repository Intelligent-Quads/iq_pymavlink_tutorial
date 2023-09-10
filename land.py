import argparse
from pymavlink import mavutil
from typing import Any


def land(the_connection: mavutil.mavlink_connection, timeout: int = 10) -> int:
    """
    Sends a command for the drone to land.

    Args:
        the_connection (mavutil.mavlink_connection): The MAVLink connection to use.
        timeout (int): Time in seconds to wait for an acknowledgment.

    Returns:
        int: mavutil.mavlink.MAV_RESULT enum value.
    """

    # Send a command to land
    the_connection.mav.command_long_send(
        the_connection.target_system, 
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait for the acknowledgment
    ack = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack is None:
        print('No acknowledgment received within the timeout period.')
        return None

    return ack.result



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--connection_string', default='udpin:localhost:14551', help='Connection string for MAVLink.')
    parser.add_argument('--timeout', type=int, default=10, help='Timeout in seconds to wait for a command acknowledgment.')
    args = parser.parse_args()

    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection(args.connection_string)

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()

    # Call the land function
    land(the_connection, args.timeout)
