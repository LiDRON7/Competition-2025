import time
from pymavlink import mavutil

# Connect to the MAVProxy instance running on the rover's autopilot
mav = mavutil.mavlink_connection('/dev/ttyUSB0', baud = 57600)  # Adjust the IP and port as necessary
print("Connected")
# Arm the rover
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("Arm succesfully")

# Run forward for 5 seconds
time.sleep(5)

# Disarm the rover
mav.mav.command_long_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)

print("Mission completed. Rover disarmed.")
