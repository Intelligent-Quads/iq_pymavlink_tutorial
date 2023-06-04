import sys
import unittest
from pymavlink import mavutil
import time
# Add the parent directory to the system path
sys.path.append('..')

from takeoff import takeoff
from sitl_simulator import SITLSimulator

class TestTakeoff(unittest.TestCase):
    def setUp(self):
        self.simulator = SITLSimulator()
        conn_str = self.simulator.start()
        time.sleep(10)
        self.mav_connection = mavutil.mavlink_connection(conn_str)

    def tearDown(self):
        self.simulator.stop()

    def test_takeoff(self):
        # Use a takeoff altitude of 10 for the test
        result = takeoff(self.mav_connection, 10)
        self.assertEqual(result, 0)

if __name__ == "__main__":
    unittest.main()
