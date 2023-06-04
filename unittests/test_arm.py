import sys
import time
import unittest
from pymavlink import mavutil

# Add the parent directory to the system path
# sys.path.append('..')

from arm import arm
from sitl_simulator import SITLSimulator

class TestArm(unittest.TestCase):
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

if __name__ == "__main__":
    unittest.main()
