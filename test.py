from pymavlink import mavutil
import time

# Connect to the vehicle
print("Connecting to the drone...")
master = mavutil.mavlink_connection('udp:172.26.80.1:14550')  # Replace with your connection string
master.wait_heartbeat()
print("Connection established! (System %u component %u )" % (master.target_system,master.target_component) ) 

#reciving messages
# while 1:
#     msg= master.recv_match(type='ATTITUDE' blocking=True)
#     print(msg) 

#arm
master.mav.command_long_send(master.target_system, master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1, 0, 0, 0, 0, 0, 0)

msg= master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)

master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    master.mode_mapping()['GUIDED']
)



master.mav.command_long_send(master.target_system, master.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0, 0, 0, 10)

msg= master.recv_match(type='COMMAND_ACK',blocking=True)
print(msg)
