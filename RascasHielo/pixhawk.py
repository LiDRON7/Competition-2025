import dronekit as dk
import time
import socket
# import exceptions
import math
import argparse


def connectMyCopter():
    baud_rate = 57600
    vehicle = dk.connect('/dev/ttyAMA0', baud=baud_rate, wait_ready=True)
    return vehicle

def arm(vehicle):
    while (vehicle.is_armable==False):
        print ("Waiting for vehicle to become armable . . .")
        time.sleep(1)

    print("Vehicle now ARMABLE\n")
    vehicle.armed = True
    
    while (vehicle.armed ==False):
        print("Waiting for drone to become armed . . . ")
        time.sleep(1)
    
    print("Vehicle now ARMED.")

    return None


def lower_drone (target_altitude, vehicle):
    vehicle.mode = dk.VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        time.sleep(1)
    vehicle.simple_takeoff (target_altitude)


def scan_mode (original_altitude, vehicle):
    vehicle.mode = "LOITER"
    while vehicle.mode != 'LOITER':
        time.sleep(1)
    vehicle.simple_goto(dk.LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, original_altitude))

#############################################################################################

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the earth's poles.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_location_coords(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobalRelative object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The function converts the distances from metres to geographical coordinates.
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return dk.LocationGlobalRelative(newlat, newlon, altitude)


def go_to_location(dNorth, dEast, altitude, vehicle):
    current_location = vehicle.location.global_relative_frame

    fixed_dnorth, fixed_deast = local_to_global(dNorth, dEast, vehicle.heading)

    target_location = get_location_coords(current_location, fixed_dnorth, fixed_deast, altitude)
    vehicle.simple_goto(target_location)

    while vehicle.mode.name == "GUIDED":
        remaining = get_distance_metres(vehicle.location.global_relative_frame, target_location)
        print("Distance to target: ", remaining)
        if remaining < 0.5:
            print("Reached target")
            break
        time.sleep(2)

    # Monitor the distance until the target is reached
    # while vehicle.location.global_relative_frame.distance_3d(target_location) > 1:
    #     print("Moving towards target...")
    #     time.sleep(2)

		
def local_to_global(x, y, heading_degrees):

    # vehicle.heading devuelve el angulo al que el vehiculo esta mirando. 0 grados es norte (magnetico) y 180 es sur
    """
    Convert local coordinates (x, y) relative to the drone's current heading into global coordinates (dNorth, dEast).
    x, y are the distances from an Aruco marker in meters.
    heading_degrees is the current heading of the drone in degrees.
    """
    # Convert heading from degrees to radians
    theta = math.radians(heading_degrees)
    
    # Calculate global coordinates
    dNorth = y * math.cos(theta) - x * math.sin(theta)
    dEast = y * math.sin(theta) + x * math.cos(theta)
    
    return dNorth, dEast

def change_height(vehicle, target_altitude):
    vehicle.simple_goto(dk.LocationGlobalRelative(vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, target_altitude))
    while vehicle.location.global_relative_frame.alt < target_altitude-0.5:
        print("Moving to target altitude...")
        time.sleep(1)
    print("Reached target altitude")

def guided (vehicle):
    print("Checking current mode: %s" % vehicle.mode.name)
    if vehicle.mode!= dk.VehicleMode("GUIDED"):
        print("Changing mode to GUIDED")
        vehicle.mode = dk.VehicleMode("GUIDED")

        while vehicle.mode != dk.VehicleMode("GUIDED"):
            print("Waiting for mode change...")
            time.sleep(1)

def auto(vehicle):
    print("Checking current mode: %s" % vehicle.mode.name)
    if vehicle.mode!= dk.VehicleMode("AUTO"):
        print("Changing mode to AUTO")
        vehicle.mode = dk.VehicleMode("AUTO")

        while vehicle.mode != dk.VehicleMode("AUTO"):
            print("Waiting for mode change...")
            time.sleep(1)

##############################
# vehicle = connectMyCopter()
# vehicle.location.global_relative_frame.alt
# vehicle.location.global_relative_frame.distance_3d()

# arm(vehicle)

# print ("Done")