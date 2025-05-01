import drone
import time

drone = drone.Drone()
print("Drone Connected")

drone.arm()
print("Arm succesfully")

pos = drone.get_position()
print(pos)

time.sleep(5)

print("Mission completed. Drone disarmed")
