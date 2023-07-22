
from pymavlink import mavutil
import time

def connect_to_sysid(connection_str : str, sysid : int, timeout: float = 3) -> any:
    """connect_to_sysid connects to a mavlink stream with a specific sysid

    Args:
        connection_str (str, optional): _description_. Defaults to 3)->mavutil.mavlink_connection(.

    Returns:
        _type_: _description_
    """    
    time_start = time.time()
    while time.time() - time_start < timeout:
        the_connection = mavutil.mavlink_connection(connection_str)
        the_connection.wait_heartbeat()
        print(
            f"Heartbeat from system system {the_connection.target_system} component {the_connection.target_component}")
        if the_connection.target_system == sysid:
            print(f"Now connected to SYSID {sysid}")
            return the_connection