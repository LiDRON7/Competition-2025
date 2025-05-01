import cv2
import depthai
import time
from drone import Drone  # Make sure drone.py is in the same directory
import numpy as np

# Initialize drone connection
drone = Drone()
print("✅ Drone connected")

# (Optional) Arm and take off — remove if you don't want auto flight here
drone.arm()
print("✅ Drone armed")

# Set up ArUco detection
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

# DepthAI pipeline setup
pipeline = depthai.Pipeline()

cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)
cam_rgb.setInterleaved(False)

xout_rgb = pipeline.createXLinkOut()
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

with depthai.Device(pipeline) as device:
    q_rgb = device.getOutputQueue("rgb")

    marker_detected = False  # To avoid multiple position grabs
    frame = None

    while True:
        in_rgb = q_rgb.tryGet()
        if in_rgb is not None:
            frame = in_rgb.getCvFrame()

        if frame is not None:
            corners, ids, rejected = detector.detectMarkers(frame)

            if ids is not None:
                ids = ids.flatten()
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                if 12 in ids and not marker_detected:
                    print("🎯 ArUco marker ID 12 detected")
                    lat, lon = drone.get_position()
                    print(f"📍 Drone GPS at detection: lat={lat:.7f}, lon={lon:.7f}")
                    marker_detected = True  # avoid repeating every frame

            else:
                marker_detected = False  # reset flag when marker is lost

        # Show camera feed
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

# (Optional) land and disarm
drone.disarm()
print("🛬 Drone disarmed. Done.")
cv2.destroyAllWindows()

