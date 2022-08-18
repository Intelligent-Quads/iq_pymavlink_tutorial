# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
# Wait a heartbeat before sending commands
the_connection.wait_heartbeat()

# Request all parameters
the_connection.mav.param_request_list_send(
    the_connection.target_system, the_connection.target_component
)
num_of_params = 999
param_values = {}
param_names = {}
while True:
    time.sleep(0.01)
    
    message = the_connection.recv_match(type='PARAM_VALUE', timeout=1, blocking=True)
    if message is None:
        break    
    else:
        print(message)
        param_values[message.param_index] = message.param_value
        param_names[message.param_index] = message.param_id
        num_of_params = message.param_count
        

for i in range(0, num_of_params):
    if i in param_values:
        continue
    else:
        print(f"index {i} not received. requesting")
        the_connection.mav.param_request_read_send(the_connection.target_system, the_connection.target_component,b'',i)
        message = the_connection.recv_match(type='PARAM_VALUE', blocking=True)
        param_values[i] = message.param_value
        param_names[i] = message.param_id
        print(f"received param {message.param_id} with value {message.param_value}")


param_id_pairs = {}

for i in range(0,num_of_params):
    param_id_pairs[param_names[i]] = param_values[i]

print(param_id_pairs)