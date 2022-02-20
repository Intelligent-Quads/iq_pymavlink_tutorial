from pymavlink import mavutil


def check_alt_reached(connection, alt):
    msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
    if abs(msg.z - (-alt)) < 1:
        return 1
    else:
        return 0


def check_reached(connection):
    msg = connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    print(msg)
    if (msg.wp_dist < 1):
        return 1
    else:
        return 0


# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      (the_connection.target_system, the_connection.target_component))

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

alt = 10
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 25, 0, 0, alt)

msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

while 1:
    if check_alt_reached(the_connection, alt):
        print("takeoff reached")
        break


wp_arr = []
wp_arr.append([10, 0, -10])
wp_arr.append([10, 10, -10])
wp_arr.append([0, 10, -10])
wp_arr.append([0, 0, -10])

wp_num = 0
while wp_num < len(wp_arr):
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system, the_connection.target_component,
                                                                                          mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), wp_arr[wp_num][0], wp_arr[wp_num][1], wp_arr[wp_num][2], 0, 0, 0, 0, 0, 0, 1.5, 0))
    while 1:
        if check_reached(the_connection):
            break
    wp_num = wp_num + 1

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
