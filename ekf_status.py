from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# List all the available EKF_STATUS_FLAGS
for value in mavutil.mavlink.enums['EKF_STATUS_FLAGS']:
    print(mavutil.mavlink.enums['EKF_STATUS_FLAGS'][value].name, value)

the_connection.wait_heartbeat()
msg = the_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True)

ekf_flags = msg.flags


print("current ekf status \n")

for value in mavutil.mavlink.enums['EKF_STATUS_FLAGS']:

    if value & ekf_flags:
        print(mavutil.mavlink.enums['EKF_STATUS_FLAGS']
              [value & ekf_flags].name)
print(mavutil.mavlink.enums['EKF_STATUS_FLAGS'])
print(mavutil.mavlink.enums['EKF_STATUS_FLAGS']
      (mavutil.MAVEnum('EKF_ATTITUDE')))
