import time
import rc

# Connect to the MAVProxy instance running on the rover's autopilot
rc = rc.RC()
print("Connected")

rc.arm()
print("Arm succesfully")
rc.set_speed(1/13) # speed = meter per second

# Current location and new location
pos = rc.get_position()
lat = pos[0]
lon = pos[1]
print("Current Latitude:", lat, "Current Longitude:", lon)

time.sleep(5)
# Move to desired lat and lon
new_lat = lat + 1.0
new_lon = lon + 1.0
rc.send_waypoint(new_lat, new_lon)
# print("Move from: ", lat, lon, "to: ", new_pos[0], new_pos[1])


# Run forward for 5 seconds
time.sleep(10)

# # Disarm the rover
rc.disarm()
print("Mission completed. Rover disarmed.")
