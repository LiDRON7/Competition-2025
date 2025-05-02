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
waypoint_a = (18.209722, -67.139444, 5)  # (lat, lon, alt)
print(f"🛫 Going to Waypoint A: {waypoint_a}")
drone.goto_position(*waypoint_a)

# Give it time to reach
time.sleep(15)

# Go to Waypoint B
waypoint_b = (18.209722 + 0.000015, -67.139444, 5)  # slightly different coords
print(f"🛬 Going to Waypoint B: {waypoint_b}")
drone.goto_position(*waypoint_b)

# Give it time to reach
time.sleep(15)

# Land
drone.land()

# Disarm
drone.disarm()
print("🏁 Mission completed. Drone disarmed.")

