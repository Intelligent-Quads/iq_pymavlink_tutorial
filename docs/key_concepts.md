# Key Concepts

## Multi Vehicle Operation

Mavlink supports multiple vehicles on a single stream. The system ID is used to differentiate between vehicles. The component ID is used to differentiate between components on a vehicle. The autopilots on the mavlink network are usually start at ID 1 and ascend. GCS in the network usually are enumerated starting at 255 and descend. 


## Responses to Commands 

Commands are sent to a vehicle using the MAV_CMD message. The autopilot will respond with a COMMAND_ACK message. The command is considered complete when the autopilot responds with a MAV_CMD_ACK message.

The MAV_CMD_ACK message will contain the MAV_RESULT enumeration. The MAV_RESULT enumeration will tell you if the command was accepted or rejected.
https://mavlink.io/en/messages/common.html#MAV_RESULT

```
0	MAV_RESULT_ACCEPTED
1	MAV_RESULT_TEMPORARILY_REJECTED
2	MAV_RESULT_DENIED
3	MAV_RESULT_UNSUPPORTED
4	MAV_RESULT_FAILED
5	MAV_RESULT_IN_PROGRESS
6	MAV_RESULT_CANCELLE
```

using pymavlink you can print the enum name using the 

