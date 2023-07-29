import argparse
from pymavlink import mavutil


def mavlink_connect(connection_str: str):
    """Connects to a MAVLink client.

    Args:
        connection_str: The connection string to the MAVLink client.

    Returns:
        A pymavlink MAVLink connection object.
    """
    connection = mavutil.mavlink_connection(connection_str)
    connection.wait_heartbeat()
    print(
        f"Heartbeat from system (system {connection.target_system} component {connection.target_component})"
    )
    return connection


def set_speed(connection, speed: float):
    """Set speed of MAVLink client.

    Args:
        connection: A pymavlink MAVLink connection object.
        speed: The speed to set.
    """
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,0,1,speed,0,0,0,0,0,)

    set_speed_ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Set Speed ACK:  {set_speed_ack}")
    return set_speed_ack.result



def set_yaw(connection, yaw: float, yaw_rate: float, direction: int = -1, abs_rel_flag: int = 0):
    """Set yaw of MAVLink client.

    Args:
        connection: A pymavlink MAVLink connection object.
        yaw: The yaw angle to set.
        yaw_rate: The yaw rate to set.
        direction: The direction to set. -1 for left, 1 for right.
        abs_rel_flag: The absolute/relative flag to set. 0 for absolute, 1 for relative.
    """
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,yaw,yaw_rate,direction,abs_rel_flag,0,0,0)

    set_yaw_ack = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Set Yaw ACK:  {set_yaw_ack}")
    return set_yaw_ack.result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Set speed and yaw of a MAVLink client"
    )
    parser.add_argument("--connection", type=str, default="udpin:localhost:14551", help="The connection string to the MAVLink client")
    parser.add_argument("--speed", type=float, default=None, help="The speed to set")
    parser.add_argument("--yaw", type=float, default=None, help="The yaw angle to set")
    parser.add_argument("--yaw-rate", type=float, default=30, help="The yaw rate to set")
    parser.add_argument("--direction", type=int, choices=[-1, 1], default=-1, help="The direction to set. -1 for left, 1 for right.")
    parser.add_argument("--abs-rel-flag", type=int, choices=[0, 1], default=0, help="The absolute/relative flag to set. 0 for absolute, 1 for relative.")
    args = parser.parse_args()

    mav_connection = mavlink_connect(args.connection)

    if args.speed is not None:
        set_speed(mav_connection, args.speed)
    if args.yaw is not None and args.yaw_rate is not None:
        set_yaw(mav_connection, args.yaw, args.yaw_rate, args.direction, args.abs_rel_flag)
