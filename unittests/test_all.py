import sys
import time
import unittest
from pymavlink import mavutil

# Add the parent directory to the system path
sys.path.append('..')

from arm import arm
from sitl_simulator import SITLSimulator
from change_mode import connect_to_sysid, change_mode  
from takeoff import takeoff

class TestAll(unittest.TestCase):
    def setUp(self):
        print("Starting simulator")
        self.simulator = SITLSimulator()
        conn_str = self.simulator.start()
        time.sleep(10)
        self.mav_connection = mavutil.mavlink_connection(conn_str)

    def tearDown(self):
        print("Stopping simulator")
        self.simulator.stop()

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

if __name__ == "__main__":
    unittest.main()
