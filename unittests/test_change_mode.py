import sys
import unittest
from pymavlink import mavutil

# Add the parent directory to the system path
# sys.path.append('..')

from change_mode import connect_to_sysid, change_mode  
from sitl_simulator import SITLSimulator

class TestChangeMode(unittest.TestCase):
    def setUp(self):
        self.simulator = SITLSimulator()
        conn_str = self.simulator.start()
        self.mav_connection = connect_to_sysid(conn_str, sysid=1)

    def tearDown(self):
        self.simulator.stop()

    def test_change_mode(self):
        # Use a mode of "GUIDED" for the test
        result = change_mode(self.mav_connection, 'GUIDED')
        
        # Check the mode of the simulated drone
        self.assertEqual(result, 0)

if __name__ == "__main__":
    unittest.main()
