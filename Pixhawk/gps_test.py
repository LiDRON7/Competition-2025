import drone
import time

drone = drone.Drone()
print("Drone Connected")

drone.arm()
print("Arm succesfully")

pos = drone.get_position()
print(pos)

drone.disarm()

print("Mission completed. Drone disarmed")
