from socket import timeout
from pymavlink import mavutil
import time


def set_param(con, param_name, value, sysid):
    # Request parameter
    con.mav.param_request_read_send(
        sysid, con.target_component,
        str.encode(param_name),
        -1
    )

    # Print old parameter value
    msg = con.recv_match(type='PARAM_VALUE',
                         blocking=True, timeout=3).to_dict()
    print(msg)
    print('name: %s\tvalue: %d' %
          (msg['param_id'], msg['param_value']))

    con.mav.param_set_send(
        sysid, con.target_component,
        str.encode(param_name),
        value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    message = con.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    print('param name: %s\t now set to value: %d' %
          (message['param_id'], message['param_value']))


# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()

print("set param")
set_param(the_connection, 'FOLL_ENABLE', 1, 1)
set_param(the_connection, 'FOLL_SYSID', 2, 1)
set_param(the_connection, 'FOLL_DIST_MAX', 500, 1)
set_param(the_connection, 'FOLL_OFS_X', -10, 1)
set_param(the_connection, 'FOLL_OFS_Y', 0, 1)
set_param(the_connection, 'FOLL_OFS_Z', 0, 1)
set_param(the_connection, 'FOLL_OFS_TYPE', 1, 1)
set_param(the_connection, 'FOLL_ALT_TYPE', 1, 1)
