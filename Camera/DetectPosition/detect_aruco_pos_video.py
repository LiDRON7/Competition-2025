# first, import all necessary modules
from pathlib import Path

import cv2
import depthai
import numpy as np
import imutils


"""
This demo calculates multiple things for different scenarios.
Here are the defined reference frames:
TAG:
                A y
                |
                |
                |tag center
                O---------> x
CAMERA:
                X--------> x
                | frame center
                |
                |
                V y
F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis
The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)
We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag
    > position of the Camera in Tag axis: -R_ct.T*tvec
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f
"""

import numpy as np
import cv2
from cv2 import aruco
import sys, time, math

#--- Define Tag
id_to_find  = 12
marker_size  = 10 #- [cm]


#------------------------------------------------------------------------------
#------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
#------------------------------------------------------------------------------
# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])




#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.loadtxt(calib_path+'cameraMatrix_oak.txt', delimiter=',')
camera_distortion   = np.loadtxt(calib_path+'cameraDistortion_oak.txt', delimiter=',')

#--- 180 deg rotation matrix around the x axis
R_flip  = np.zeros((3,3), dtype=np.float32)
R_flip[0,0] = 1.0
R_flip[1,1] =-1.0
R_flip[2,2] =-1.0

#--- Define the aruco dictionary
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50, )
arucoParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(arucoDict, arucoParams)

#--- Capture the videocamera (this may also be a video or a picture)
# cap = cv2.VideoCapture(0)
# #-- Set the camera size as the one it was calibrated with
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_SIMPLEX


"""
Set up to use the oak-D start
"""
pipeline = depthai.Pipeline()

# First, we want the Color camera as the output
cam_rgb = pipeline.createColorCamera()
cam_rgb.setPreviewSize(300, 300)  # 300x300 will be the preview frame size, available as 'preview' output of the node
cam_rgb.setInterleaved(False)

# Next, we want a neural network that will produce the detections
# detection_nn = pipeline.createMobileNetDetectionNetwork()

# Blob is the Neural Network file, compiled for MyriadX. It contains both the definition and weights of the model
# We're using a blobconverter tool to retreive the MobileNetSSD blob automatically from OpenVINO Model Zoo
# detection_nn.setBlobPath(blobconverter.from_zoo(name='mobilenet-ssd', shaves=6))

# Next, we filter out the detections that are below a confidence threshold. Confidence can be anywhere between <0..1>
# detection_nn.setConfidenceThreshold(0.5)

# Next, we link the camera 'preview' output to the neural network detection input, so that it can produce detections
# cam_rgb.preview.link(detection_nn.input)


# XLinkOut is a "way out" from the device. Any data you want to transfer to host need to be send via XLink
xout_rgb = pipeline.createXLinkOut()
# For the rgb camera output, we want the XLink stream to be named "rgb"
xout_rgb.setStreamName("rgb")
# Linking camera preview to XLink input, so that the frames will be sent to host
cam_rgb.preview.link(xout_rgb.input)

# The same XLinkOut mechanism will be used to receive nn results
xout_nn = pipeline.createXLinkOut()
xout_nn.setStreamName("nn")
# detection_nn.out.link(xout_nn.input)
"""
Set up to use the oak-D end
"""
with depthai.Device(pipeline) as device:
    # From this point, the Device will be in "running" mode and will start sending data via XLink

    # To consume the device results, we get two output queues from the device, with stream names we assigned earlier
    q_rgb = device.getOutputQueue("rgb")
    q_nn = device.getOutputQueue("nn")

    # Here, some of the default values are defined. Frame will be an image from "rgb" stream, detections will contain nn results
    frame = None
    detections = []

    # Since the detections returned by nn have values from <0..1> range, they need to be multiplied by frame width/height to
    # receive the actual position of the bounding box on the image
    def frameNorm(frame, bbox):
        normVals = np.full(len(bbox), frame.shape[0])
        normVals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)


    while True:
        # we try to fetch the data from nn/rgb queues. tryGet will return either the data packet or None if there isn't any
        in_rgb = q_rgb.tryGet()
        in_nn = q_nn.tryGet()

        if in_rgb is not None:
            # If the packet from RGB camera is present, we're retrieving the frame in OpenCV format using getCvFrame
            frame = in_rgb.getCvFrame()

        if in_nn is not None:
            # when data from nn is received, we take the detections array that contains mobilenet-ssd results
            detections = in_nn.detections
        #-- Read the camera frame
        if frame is None:
            continue
        #-- Find all the aruco markers in the image
        corners, ids, rejected_candidates = detector.detectMarkers(frame)
        
        if len(corners) > 0:
            
            # Asume que marker_size está en centímetros y es un cuadrado (por ejemplo, 10cm)
            half_size = marker_size / 2.0

            # Puntos del marcador en coordenadas 3D (origen en el centro del marcador)
            marker_corners_3d = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            # corners[0] son los 4 puntos del marcador detectado, en 2D
            image_points = corners[0].reshape(-1, 2)

            # Resolver pose
            success, rvec, tvec = cv2.solvePnP(marker_corners_3d, image_points, camera_matrix, camera_distortion)
            rvec = rvec.reshape(3)
            tvec = tvec.reshape(3)


            #-- Draw the detected marker and put a reference frame over it
            aruco.drawDetectedMarkers(frame, corners)
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)



            print("===================")
            #-- Print the tag position in camera frame
            str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
            cv2.putText(frame, str_position, (0, 20), font, .35, (255,0,127), 2, cv2.LINE_AA)
            print(f"MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2]))

            #-- Obtain the rotation matrix tag->camera
            R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
            roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

            #-- Print the marker's attitude respect to camera frame
            str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker))
            cv2.putText(frame, str_attitude, (0, 40), font, .35, (255,0,127), 2, cv2.LINE_AA)
            print("MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                math.degrees(yaw_marker)))

            #-- Now get Position and attitude f the camera respect to the marker
            pos_camera = -R_tc*np.matrix(tvec).T

            str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
            cv2.putText(frame, str_position, (0, 60), font, .35, (255,0,127), 2, cv2.LINE_AA)
            print("CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2]))

            #-- Get the attitude of the camera respect to the frame
            roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
            str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera))
            cv2.putText(frame, str_attitude, (0, 80), font, .35, (255,0,127), 2, cv2.LINE_AA)
            print("CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                math.degrees(yaw_camera)))

            print("===================")
        


        #--- Display the frame
        cv2.imshow('frame', frame)

        #--- use 'q' to quit
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyAllWindows()
            break


