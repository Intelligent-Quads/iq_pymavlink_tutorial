from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
print(msg)

print(mavutil.mavlink.enums['MAV_RESULT'][msg.result].description)
