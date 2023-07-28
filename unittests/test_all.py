import sys
import time
import unittest
import os
from pymavlink import mavutil

# Add the parent directory to the system path
sys.path.append('..')

from arm import arm
from sitl_simulator import SITLSimulator
from change_mode import connect_to_sysid, change_mode  
from takeoff import takeoff
from speed_yaw import set_speed, set_yaw
from upload_waypoints import upload_qgc_mission
from land import land


class TestAll(unittest.TestCase):
    def setUp(self):
        print("Starting simulator")
        self.simulator = SITLSimulator()
        conn_str = self.simulator.start()
        time.sleep(10)
        self.mav_connection = self.connect_to_sysid(conn_str, 1)
        self.mav_connection.mav.request_data_stream_send(self.mav_connection.target_system, self.mav_connection.target_component,
                                        mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    def tearDown(self):
        print("Stopping simulator")
        self.simulator.stop()
        self.mav_connection.close()

    def connect_to_sysid(self, connection_str : str, sysid : int, timeout: float = 3) -> any:
        """connect_to_sysid connects to a mavlink stream with a specific sysid

        Args:
            connection_str (str, optional): _description_. Defaults to 3)->mavutil.mavlink_connection(.

        Returns:
            _type_: _description_
        """    
        time_start = time.time()
        while time.time() - time_start < timeout:
            the_connection = mavutil.mavlink_connection(connection_str, autoreconnect=True)
            the_connection.wait_heartbeat()
            print(
                f"Heartbeat from system system {the_connection.target_system} component {the_connection.target_component}")
            if the_connection.target_system == sysid:
                print(f"Now connected to SYSID {sysid}")
                return the_connection


    def test_arm(self):
        result = arm(self.mav_connection, 1)
        self.assertEqual(result, 0)

    def test_change_mode(self):
        # Use a mode of "GUIDED" for the test
        result = change_mode(self.mav_connection, 'GUIDED')
        
        # Check the mode of the simulated drone
        self.assertEqual(result, 0)
    
    def test_takeoff(self):
        # Use a takeoff altitude of 10 for the test
        result = takeoff(self.mav_connection, 10)
        self.assertEqual(result, 0)

    def test_set_speed(self):
        result = takeoff(self.mav_connection, 10)
        self.assertEqual(result, 0)
        time.sleep(1)
        # Use a speed of 1 for the test
        result = set_speed(self.mav_connection, 1)

        self.assertEqual(result, 0)

    def test_set_yaw(self):
        result = takeoff(self.mav_connection, 10)
        self.assertEqual(result, 0)
        time.sleep(1)

        # Use a yaw angle of 45 and a yaw rate of 25 for the test
        result = set_yaw(self.mav_connection, 45, 25)
        self.assertEqual(result, 0)

    def test_upload_waypoints(self):
        # Use CMAC_square.plan file for the test
        mission_file = os.path.join(os.path.dirname(__file__), "..", "wps", "CMAC_square.plan")
        result = upload_qgc_mission(mission_file, self.mav_connection)
        self.assertTrue(result)
    
    def test_land(self):
        result = takeoff(self.mav_connection, 10)
        self.assertEqual(result, 0)
        time.sleep(10)
        result = land(self.mav_connection)
        self.assertEqual(result, 0)

if __name__ == "__main__":
    unittest.main()
