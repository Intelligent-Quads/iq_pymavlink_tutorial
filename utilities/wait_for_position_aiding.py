import time
import sys
import os
from pymavlink import mavutil

# add this folder to the path
utilities_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(utilities_path)
from get_autopilot_info import get_autopilot_info


def get_enum_value_by_name(enum_dict, name):
    for key, enum_entry in enum_dict.items():
        if enum_entry.name == name:
            return key
    raise ValueError("No enum entry with name: " + name)


def wait_until_position_aiding(mav_connection):
    autopilot_info = get_autopilot_info(mav_connection)
    if autopilot_info["autopilot"] == "ardupilotmega":
        estimator_status_msg = "EKF_STATUS_REPORT"
    elif autopilot_info["autopilot"] == "px4":
        estimator_status_msg = "ESTIMATOR_STATUS"
    else:
        raise ValueError("Autopilot not supported")

    flags = ["EKF_PRED_POS_HORIZ_REL", "EKF_PRED_POS_HORIZ_REL"]
    time_start = time.time()
    while (
        not ekf_pos_aiding(mav_connection, flags, estimator_status_msg)
        and time.time() - time_start < 120
    ):
        print(time.time() - time_start)
        continue
    return


def ekf_pos_aiding(mav_connection, flags, estimator_status_msg="ESTIMATOR_STATUS"):
    # msg = mav_connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=3)
    msg = mav_connection.recv_match(type=estimator_status_msg, blocking=True, timeout=3)
    if not msg:
        return 0
    print(f"from sysid {msg.get_srcSystem()} {msg}")
    ekf_flags = msg.flags

    for flag in flags:
        flag_val = get_enum_value_by_name(
            mavutil.mavlink.enums["EKF_STATUS_FLAGS"], flag
        )
        if ekf_flags & flag_val:
            continue
        else:
            return 0

    return 1
