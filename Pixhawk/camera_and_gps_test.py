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

# === THREADING SETUP ===
marker_event = threading.Event()  # Used to signal detection
stop_event = threading.Event()    # Used to stop the camera loop gracefully

# === CAMERA PIPELINE FUNCTION ===
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
def camera_loop(marker_event, stop_event):
    camera_attempts = 0
    max_attempts = 3

    while camera_attempts < max_attempts and not marker_event.is_set():
        try:
            print(f"🎥 Attempting to start camera (try #{camera_attempts + 1})")
            pipeline = create_camera_pipeline()
            with depthai.Device(pipeline) as device:
                q_rgb = device.getOutputQueue("rgb")
                print("✅ Camera started. Searching for marker...")

                while not stop_event.is_set():
                    in_rgb = q_rgb.tryGet()
                    if in_rgb is not None:
                        frame = in_rgb.getCvFrame()
                        corners, ids, _ = detector.detectMarkers(frame)

                        if ids is not None and 14 in ids.flatten():
                            print("🎯 ArUco ID 14 detected!")
                            marker_event.set()
                            return
                    time.sleep(0.05)
            break
        except Exception as e:
            camera_attempts += 1
            print(f"❌ Camera error (attempt {camera_attempts}): {e}")
            time.sleep(2)

# === MAIN DRONE FLOW ===
drone = Drone()
drone.set_mode("GUIDED")
drone.arm()
drone.takeoff(altitude=5)
print("🛑 Staying in GUIDED mode for marker search...")

# === Start Camera Thread ===
cam_thread = threading.Thread(target=camera_loop, args=(marker_event, stop_event))
cam_thread.start()

try:
    while not marker_event.is_set():
        print("🔎 Waiting for marker detection...")
        time.sleep(1)

    print("📍 Marker detected! Initiating landing...")
    lat, lon = drone.get_position()
    print(f"🌐 Drone position: lat={lat:.7f}, lon={lon:.7f}")
    drone.set_mode("GUIDED")
    drone.land()

except KeyboardInterrupt:
    print("🛑 Interrupted by user. Landing...")
    drone.land()

finally:
    stop_event.set()
    cam_thread.join()
    drone.disarm()
    print("✅ Drone disarmed. Mission ended.")
    cv2.destroyAllWindows()

