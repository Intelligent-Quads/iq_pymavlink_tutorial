# sitl_simulator.py
import os
import iq_sim
from pymavlink import mavutil

class SITLSimulator:
    def __init__(self):
        self.token = os.getenv("IQ_SIM_TOKEN")
        self.api = iq_sim.iq_sim(self.token)

    def start(self):
        self.sim_id = self.api.start_sim()
        self.api.wait_for_sim_ready(self.sim_id)
        self.connection_info = self.api.get_connection(self.sim_id)
        print(self.connection_info)
        self.conn_str = f"tcp:{self.connection_info['ip']}:{self.connection_info['port']}"
        return self.conn_str

    def stop(self):
        self.api.stop_sim(self.sim_id)
