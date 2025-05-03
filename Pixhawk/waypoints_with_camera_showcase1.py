import cv2
import depthai
import time
import threading
from drone import Drone
import numpy as np

# === ARUCO SETUP ===
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# === THREAD FLAGS ===
marker_event = threading.Event()
stop_event = threading.Event()

aruco_number = 13

altitude_sl = 3

# === CAMERA PIPELINE ===
def create_camera_pipeline():
    pipeline = depthai.Pipeline()
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(300, 300)
    cam_rgb.setInterleaved(False)

    xout_rgb = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)
    return pipeline

# === CAMERA THREAD FUNCTION ===
def camera_loop(marker_event, stop_event, max_attempts=5):
    attempts = 0
    
    while attempts < max_attempts and not stop_event.is_set() and not marker_event.is_set():
        try:
            print(f"🎥 Attempting to start camera (try #{attempts + 1})")
            pipeline = create_camera_pipeline()
            with depthai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue("rgb")
                print("✅ Camera started successfully.")
                while not stop_event.is_set() and not marker_event.is_set():
                    in_rgb = q_rgb.tryGet()
                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        corners, ids, _ = detector.detectMarkers(frame)
                        if ids is not None and aruco_number in ids.flatten():
                            print(f"ArUco marker {aruco_number} detected! - FOUND ARUCO")
                            marker_event.set()
                            return

                        elif ids is not None and len(ids.flatten()) > 0:
                            print(f"Aruco marker {ids.flatten()[0]} detected - Not the one")

                    time.sleep(0.1)
                return  # End after stop or detection
        except Exception as e:
            attempts += 1
            print(f"❌ Camera error (attempt {attempts}): {e}")
            if attempts < max_attempts:
                print("🔁 Retrying camera initialization...")
                time.sleep(2)
            else:
                print("🛑 Camera failed multiple times. Aborting mission.")
                marker_event.set()

# === FLIGHT THREAD FUNCTION ===
def flight_path(drone, marker_event):
    waypoints = [
        (18.3932975, -66.1510085, altitude_sl),  # Example waypoints (lat, lon, alt)
        (18.3932911, -66.1510058, altitude_sl),
        (18.3932956, -66.1509991, altitude_sl),
        (18.3933008, -66.1510019, altitude_sl),
        (18.3932929, -66.1510002, altitude_sl),
        (18.3932498, -66.1509541, altitude_sl),
        (18.3932345, -66.1509568, altitude_sl),
        (18.3932271, -66.1509563, altitude_sl),
        (18.3932377, -66.1509464, altitude_sl),
        (18.3932415, -66.1509676, altitude_sl),
        (18.3932774, -66.1510514, altitude_sl),
        (18.3932790, -66.1510665, altitude_sl),
        (18.3932695, -66.1510470, altitude_sl),
        (18.3932679, -66.1510611, altitude_sl),
        (18.3932682, -66.1510661, altitude_sl),
        (18.3933700, -66.1510433, altitude_sl),
        (18.3933675, -66.1510574, altitude_sl),
        (18.3933808, -66.1510527, altitude_sl),
        (18.3933586, -66.1510440, altitude_sl),
        (18.3933644, -66.1510547, altitude_sl),
        (18.3933781, -66.1510450, altitude_sl),
        (18.3933516, -66.1509468, altitude_sl),
        (18.3933516, -66.1509347, altitude_sl),
        (18.3933420, -66.1509421, altitude_sl),
        (18.3933528, -66.1509410, altitude_sl),
        (18.3933455, -66.1509329, altitude_sl),
    ]

    print("🚀 Starting mission with waypoints...")
    for lat, lon, alt in waypoints:
        if marker_event.is_set():
            print("⚠️ Marker detected during mission. Aborting path...")
            break

        print(f"📍 Going to waypoint: {lat}, {lon}, {alt}")
        drone.goto_position(lat, lon, alt)
        time.sleep(5)  # Give it time to reach the point

    if marker_event.is_set():
        drone.land()
    else:
        print("✅ Mission complete. Returning to launch.")
        drone.land()

# === MARKER LISTENER THREAD FUNCTION ===
def marker_listener(drone, marker_event, stop_event):
    while not stop_event.is_set():
        if marker_event.is_set():
            position = drone.get_position()  # Assumes this returns (lat, lon, alt) or similar
            print(f"📡 ArUco detected! Drone position: {position}")
            return  # Exit thread after handling once
        time.sleep(0.2)

# === MAIN LOGIC ===

start_time = time.time()

drone = Drone()

# Set geofence boundaries here
#drone.set_geofence(
#    min_lat=18.4654000, max_lat=18.4660000,
#    min_lon=-66.1060000, max_lon=-66.1050000,
#    min_alt=3, max_alt=20
#)

drone.set_mode("GUIDED")
drone.arm()
drone.takeoff(altitude=altitude_sl)

cam_thread = threading.Thread(target=camera_loop, args=(marker_event, stop_event))
flight_thread = threading.Thread(target=flight_path, args=(drone, marker_event))
marker_listener_thread = threading.Thread(target=marker_listener, args=(drone, marker_event, stop_event))

cam_thread.start()
flight_thread.start()
marker_listener_thread.start()

flight_thread.join()
stop_event.set()
cam_thread.join()
marker_listener_thread.join()

drone.disarm()
print("✅ Drone disarmed. Mission complete.")
cv2.destroyAllWindows()

end_time = time.time()  # End the timer
elapsed = end_time - start_time
print(f"⏱️ Total mission duration: {elapsed:.2f} seconds.")
