import time
import sys
import os
from pymavlink import mavutil

# add this folder to the path
utilities_path = os.path.dirname(os.path.abspath(__file__))
sys.path.append(utilities_path)
from get_autopilot_info import get_autopilot_info


def get_enum_value_by_name(enum_dict, name):
    """
    Get the value of an enum entry by its name.

    Args:
        enum_dict (Dict[str, Enum]): The enum dictionary to search.
        name (str): The name of the enum entry.

    Returns:
        int: The value of the enum entry.

    Raises:
        ValueError: If no enum entry with the given name is found.
    """
    for key, enum_entry in enum_dict.items():
        if enum_entry.name == name:
            return key
    raise ValueError("No enum entry with name: " + name)


def wait_until_position_aiding(mav_connection, timeout=120):
    """
    Wait until the MAVLink connection has EKF position aiding.

    Args:
        mav_connection (mavutil.mavlink_connection): The MAVLink connection to check.
        timeout (int, optional): The maximum time to wait for EKF position aiding in seconds. Defaults to 120.

    Raises:
        TimeoutError: If EKF position aiding is not achieved within the specified timeout.

    Returns:
        None
    """
    autopilot_info = get_autopilot_info(mav_connection)
    if autopilot_info["autopilot"] == "ardupilotmega":
        estimator_status_msg = "EKF_STATUS_REPORT"
    elif autopilot_info["autopilot"] == "px4":
        estimator_status_msg = "ESTIMATOR_STATUS"
    else:
        raise ValueError("Autopilot not supported")

    flags = ["EKF_PRED_POS_HORIZ_REL", "EKF_PRED_POS_HORIZ_REL"]
    time_start = time.time()
    while True:
        if ekf_pos_aiding(mav_connection, flags, estimator_status_msg) or time.time() - time_start > timeout:
            break
        print(f"Waiting for position aiding: {time.time() - time_start} seconds elapsed")

    if time.time() - time_start > timeout:
        raise TimeoutError(f"Position aiding did not become available within {timeout} seconds")



def ekf_pos_aiding(mav_connection, flags, estimator_status_msg="ESTIMATOR_STATUS"):
    """
    Check the EKF position aiding status of a MAVLink connection.

    Args:
        mav_connection (mavutil.mavlink_connection): The MAVLink connection to check.
        flags (List[str]): The flags to check in the EKF status.
        estimator_status_msg (str, optional): The name of the estimator status message. Defaults to "ESTIMATOR_STATUS".

    Returns:
        bool: True if all flags are present in the EKF status, False otherwise.
    """
    msg = mav_connection.recv_match(type=estimator_status_msg, blocking=True, timeout=3)
    if not msg:
        raise ValueError(f"No message of type {estimator_status_msg} received within the timeout")

    print(f"from sysid {msg.get_srcSystem()} {msg}")
    ekf_flags = msg.flags

    for flag in flags:
        flag_val = get_enum_value_by_name(mavutil.mavlink.enums["EKF_STATUS_FLAGS"], flag)
        if not ekf_flags & flag_val:
            return False

    return True
