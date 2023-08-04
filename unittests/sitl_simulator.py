# sitl_simulator.py
import os
import iq_sim
from pymavlink import mavutil

class SITLSimulator:
    def __init__(self):
        self.token = os.getenv("IQ_SIM_TOKEN")
        self.api = iq_sim.iq_sim(self.token)

    def start(self):

        # read autiopilot version from env variable
        autopilot = os.getenv("IQ_SIM_AUTOPILOT", "ardupilot")
        if autopilot == "ardupilot":
            sim_config = self.api.sim_config
        elif autopilot == "px4":
            sim_config = {
                "sim_config": [
                    {
                        "sim_type": "jmavsim",
                        "vehicle_type": "quadcopter",
                        "vehicle_model": "quad",
                        "instances": "1",
                        "flight_controls": "PX4",
                        "fc_version": self.api.sim_capabilities["flight_controls"]["PX4"]["simulation"]["jmavsim"]["vehicle_types"]["quadcopter"]["version_default"],
                        "latlonaltheading": [
                            "-35.363261",
                            "149.16523",
                            "584",
                            "353"
                        ]
                    }
                ]
            }

        self.sim_id = self.api.start_sim(sim_config)
        try:
            self.api.wait_for_sim_ready(self.sim_id)
            self.connection_info = self.api.get_connection(self.sim_id)
            print(self.connection_info)
            self.conn_str = f"tcp:{self.connection_info['ip']}:{self.connection_info['port']}"
            return self.conn_str
        except:
            self.api.stop_sim(self.sim_id)
            raise

    def stop(self):
        self.api.stop_sim(self.sim_id)
