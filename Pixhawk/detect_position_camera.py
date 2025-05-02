import cv2
import depthai
import time
from drone import Drone
import numpy as np

# === DRONE SETUP ===
drone = Drone()
drone.set_mode("GUIDED")
drone.arm()
drone.takeoff(altitude=10)
drone.set_mode("LOITER")  # Hover until marker detected
drone.loitering(altitude=10, seconds=10)

# === ARUCO SETUP ===
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# === CAMERA PIPELINE DEFINITION ===

def create_camera_pipeline():
    pipeline = depthai.Pipeline()
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(300, 300)
    cam_rgb.setInterleaved(False)

    xout_rgb = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)
    return pipeline

# === CAMERA HANDLING ===
camera_attempts = 0
max_attempts = 2
marker_detected = False

time.sleep(2)
while camera_attempts < max_attempts and not marker_detected:
    try:
        print(f"🎥 Attempting to start camera (try #{camera_attempts + 1})")
        pipeline = create_camera_pipeline()
        with depthai.Device(pipeline) as device:
            q_rgb = device.getOutputQueue("rgb")
            print("✅ Camera started. Drone is loitering...")

            while True:
                in_rgb = q_rgb.tryGet()
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                    corners, ids, rejected = detector.detectMarkers(frame)

                    if ids is not None and 14 in ids.flatten():
                        print("🎯 ArUco ID 14 detected!")
                        lat, lon = drone.get_position()
                        print(f"📍 GPS: lat={lat:.7f}, lon={lon:.7f}")

                        drone.set_mode("GUIDED")
                        drone.land()
                        marker_detected = True
                        break

                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break
        break  # Camera session completed successfully
    except Exception as e:
        camera_attempts += 1
        print(f"❌ Camera error (attempt {camera_attempts}): {e}")
        if camera_attempts < max_attempts:
            print("🔁 Retrying camera initialization...")
            time.sleep(2)
        else:
            print("🛑 Camera failed twice. Landing drone for safety...")
            drone.set_mode("GUIDED")
            drone.land()

# Always disarm and cleanup
drone.set_mode("GUIDED")
drone.land()
drone.disarm()
cv2.destroyAllWindows()
print("✅ Drone disarmed. Mission ended.")

