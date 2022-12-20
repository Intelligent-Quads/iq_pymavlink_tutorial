# Import mavutil
from pymavlink import mavutil

# Create the connection to the top-side computer as companion computer/autopilot
the_connection = mavutil.mavlink_connection('udpout:localhost:14550', source_system=1)

# Send a message for QGC to read out loud
#  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY


the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_EMERGENCY, "This is an emergency".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_ALERT, "This is an alert".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_CRITICAL, "This is critical".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_ERROR, "This is an error".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING, "This is a warning".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE, "This is a notice".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, "This is an info".encode())

the_connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_DEBUG, "This is a debug".encode())

