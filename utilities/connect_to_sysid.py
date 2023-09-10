from pymavlink import mavutil
import time

def connect_to_sysid(connection_str : str, sysid : int, timeout: float = 3) -> any:
    """
    connect_to_sysid connects to a mavlink stream with a specific sysid

    Args:
        connection_str (str): String containing the connection information
        sysid (int): The system id to connect to
        timeout (float, optional): Maximum time to wait for the connection in seconds. Defaults to 3.

    Returns:
        mavutil.mavlink_connection: Returns the connection object if connection is successful, 
        else returns None after the timeout
    """    
    the_connection = mavutil.mavlink_connection(connection_str)
    time_start = time.time()

    while time.time() - time_start < timeout:
        try:
            the_connection.wait_heartbeat()
            print(f"Heartbeat from system {the_connection.target_system} component {the_connection.target_component}")
            if the_connection.target_system == sysid:
                print(f"Now connected to SYSID {sysid}")
                return the_connection
        except Exception as e:
            print(f"Error while waiting for heartbeat: {e}")
            return None

    print(f"Connection timeout after {timeout} seconds")
    return None
