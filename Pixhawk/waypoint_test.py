import time
from drone import Drone  # assuming your file is named `drone.py`

# Initialize and connect to drone
drone = Drone()
print("✅ Drone connected")

# Take off to 10 meters
drone.takeoff(altitude=5)

# Optional: wait to stabilize
time.sleep(5)

# Go to Waypoint A (example coordinates)
waypoint_a = (18.2098535, -67.1395124, 5)  # (lat, lon, alt)
print(f"🛫 Going to Waypoint A: {waypoint_a}")
drone.goto_position(*waypoint_a)

# Give it time to reach
time.sleep(15)

# Go to Waypoint B
waypoint_b = (18.2096682, -67.1395554, 5)  # slightly different coords
print(f"🛬 Going to Waypoint B: {waypoint_b}")
drone.goto_position(*waypoint_b)

# Give it time to reach
time.sleep(15)

waypoint_c = (18.2097490, -67.1397163, 5)  # slightly different coords
print(f"🛬 Going to Waypoint C: {waypoint_c}")
drone.goto_position(*waypoint_c)

time.sleep(15)
# Land
drone.land()

# Disarm
drone.disarm()
print("🏁 Mission completed. Drone disarmed.")