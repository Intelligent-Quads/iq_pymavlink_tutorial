import time
import argparse
import math
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

    msg = con.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    if msg is None:
        takeoff(con, alt, target_sysid)
        return
    
    if msg.result != 0:
        print(
            f"Error requesting takeoff drone. takeoff request returned {msg.result}")
        takeoff(con, alt, target_sysid)
        return 
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


def generate_circle_points(num_points, center=[0,0,0], radius=10):
    points = []
    for i in range(num_points):
        points.append([center[0] + radius * math.cos(2 * math.pi * i / num_points),
                       center[1] + radius * math.sin(2 * math.pi * i / num_points),
                       center[2]])
    return points




def new_main(num_vehicles, radius):
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (the_connection.target_system, the_connection.target_component))
    

    

    for i in range(num_vehicles):  
        takeoff(the_connection, 10, target_sysid=i+1)

    time.sleep(10)

    waypoint_reached = [0] * num_vehicles
    while 1:
        for i in range(num_vehicles):
            waypoint_reached[i] = check_reached(the_connection, [0,0,-10], i+1)
        if all(waypoint_reached):
            print("all aircraft at destination")
            break
    


    wps = generate_circle_points(num_vehicles, [0,0,-10], radius=radius)
    print(f"waypoints: {wps}")
    for i in range(num_vehicles):
        goto_ned(the_connection, wps[i], i+1)






if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument('--connect', default='udpin:localhost:14551')
    parser.add_argument('-i', default=1, type=int, help='number of vehicles')
    parser.add_argument('--radius', default=10, type=float, help='radius of circle')
    args = parser.parse_args()


    new_main(args.i, args.radius)