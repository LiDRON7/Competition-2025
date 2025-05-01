import drone
import time

drone = drone.Drone()
print("Drone Connected")

drone.arm()
print("Arm succesfully")

pos = drone.get_position()
print(pos)

drone.takeoff_loiter_land(altitude=10)

drone.disarm()

print("Mission completed. Drone disarmed")
