from pymavlink import mavutil

# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')


for value in mavutil.mavlink.enums['EKF_STATUS_FLAGS']:
    print(mavutil.mavlink.enums['EKF_STATUS_FLAGS'][value].name, value)
# print(mavutil.mavlink.enums['EKF_STATUS_FLAGS'][:].name)

the_connection.wait_heartbeat()
msg = the_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True)

ekf_flags = msg.flags


print("current ekf status \n")

for value in mavutil.mavlink.enums['EKF_STATUS_FLAGS']:
    # print("{0:b}".format(value))
    # print(value)

    if value & ekf_flags:
        print(mavutil.mavlink.enums['EKF_STATUS_FLAGS']
              [value & ekf_flags].name)
print(mavutil.mavlink.enums['EKF_STATUS_FLAGS'])
print(mavutil.mavlink.enums['EKF_STATUS_FLAGS']
      (mavutil.MAVEnum('EKF_ATTITUDE')))
