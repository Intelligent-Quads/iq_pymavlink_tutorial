import time
from pymavlink import mavutil


# this program assumes that there are 3 vehicles participating in coordinated flight with sys is 1 - 3


def takeoff(con, alt, target_sysid=1):
    con.mav.command_long_send(target_sysid, con.target_component,
                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = con.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result != 0:
        print(f"Error arming drone. arm request returned {msg.result}")
        return -1
    else:
        print(f"drone sysid {target_sysid} armed successfully")

    con.mav.command_long_send(target_sysid, con.target_component,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 25, 0, 0, alt)

    msg = con.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result != 0:
        print(
            f"Error requesting takeoff drone. takeoff request returned {msg.result}")
        return -1
    else:
        print(f"drone sysid {target_sysid} took off successfully")


def goto_ned(con, wp, target_sysid=1):
    con.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, target_sysid, 1,
                                                                               mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), wp[0], wp[1], wp[2], 0, 0, 0, 0, 0, 0, 1.5, 0))


def check_reached(connection, wp, target_sysid=1):

    while 1:
        msg = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        if msg.get_srcSystem() == target_sysid:
            print(msg)
            if (abs(msg.x - wp[0]) < 0.5) and (abs(msg.y - wp[1]) < 0.5) and (abs(msg.z - wp[2]) < 0.5):
                return 1
            else:
                return 0


def main():

    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))

    takeoff(the_connection, 10, target_sysid=1)

    takeoff(the_connection, 10, target_sysid=2)

    takeoff(the_connection, 10, target_sysid=3)

    takeoff(the_connection, 10, target_sysid=4)

    takeoff(the_connection, 10, target_sysid=5)

    takeoff(the_connection, 10, target_sysid=6)

    takeoff(the_connection, 10, target_sysid=7)

    time.sleep(10)

    aircraft1_wp = [10, 0, -10]
    aircraft2_wp = [10, -10, -10]
    aircraft3_wp = [10, 10, -10]
    aircraft4_wp = [0, 0, -10]
    aircraft5_wp = [-10, 0, -10]
    aircraft6_wp = [-10, -10, -10]
    aircraft7_wp = [-10, 10, -10]
    goto_ned(the_connection, aircraft1_wp, 1)
    goto_ned(the_connection, aircraft2_wp, 2)
    goto_ned(the_connection, aircraft3_wp, 3)
    goto_ned(the_connection, aircraft4_wp, 4)
    goto_ned(the_connection, aircraft5_wp, 5)
    goto_ned(the_connection, aircraft6_wp, 6)
    goto_ned(the_connection, aircraft7_wp, 7)

    aircraft1_ready = 0
    aircraft2_ready = 0
    aircraft3_ready = 0
    while 1:
        if check_reached(the_connection, aircraft1_wp, 1):
            aircraft1_ready = 1
        if check_reached(the_connection, aircraft2_wp, 2):
            aircraft2_ready = 1
        if check_reached(the_connection, aircraft3_wp, 3):
            aircraft3_ready = 1
        if check_reached(the_connection, aircraft4_wp, 4):
            aircraft4_ready = 1
        if check_reached(the_connection, aircraft5_wp, 5):
            aircraft5_ready = 1
        if check_reached(the_connection, aircraft6_wp, 6):
            aircraft6_ready = 1
        if check_reached(the_connection, aircraft7_wp, 7):
            aircraft7_ready = 1

        if aircraft1_ready and aircraft2_ready and aircraft3_ready and aircraft4_ready and aircraft5_ready and aircraft6_ready and aircraft7_ready:
            print("all aircraft at destination")
            break


if __name__ == "__main__":
    main()
