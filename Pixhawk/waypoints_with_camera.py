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

aruco_number = 14
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
                            print(f`ArUco marker {aruco_number} detected! Returning to launch...`)
                            marker_event.set()
                            return
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
        (18.4655390, -66.1057360, 10),  # Example waypoints (lat, lon, alt)
        (18.4657400, -66.1059000, 10),
        (18.4658000, -66.1055000, 10)
    ]

    print("🚀 Starting mission with waypoints...")
    for lat, lon, alt in waypoints:
        if marker_event.is_set():
            print("⚠️ Marker detected during mission. Aborting path...")
            break

        print(f"📍 Going to waypoint: {lat}, {lon}, {alt}")
        drone.goto_position(lat, lon, alt)
        time.sleep(10)  # Give it time to reach the point

    if marker_event.is_set():
        drone.return_to_launch()
    else:
        print("✅ Mission complete. Returning to launch.")
        drone.return_to_launch()

# === MAIN LOGIC ===
drone = Drone()
drone.set_mode("GUIDED")
drone.arm()
drone.takeoff(altitude=10)

cam_thread = threading.Thread(target=camera_loop, args=(marker_event, stop_event))
flight_thread = threading.Thread(target=flight_path, args=(drone, marker_event))

cam_thread.start()
flight_thread.start()

flight_thread.join()
stop_event.set()
cam_thread.join()

drone.disarm()
print("✅ Drone disarmed. Mission complete.")
cv2.destroyAllWindows()

