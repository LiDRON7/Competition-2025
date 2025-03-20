# code from https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/


# import the necessary packages
# from imutils.video import VideoStream
import argparse
# import imutils
import time
import cv2
import sys
import numpy as np
import math
from Aruco import *
import time
import json
# import pixhawk


# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str,
	default="DICT_6X6_1000",
	help="type of ArUCo tag to detect")
args = vars(ap.parse_args())

# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

# verify that the supplied ArUCo tag exists and is supported by
# OpenCV
if ARUCO_DICT.get(args["type"], None) is None:
	print("[INFO] ArUCo tag of '{}' is not supported".format(
		args["type"]))
	sys.exit(0)
	
##################################################################Get things from JSON*******
with open('calibration_data.json', 'r') as f:
  calibration_data = json.load(f)
print (calibration_data)
camera_matrix = np.array(calibration_data['camera_matrix'])
distortion_coeffs = np.array(calibration_data['distortion_coefficients'])

################################################################################

# load the ArUCo dictionary and grab the ArUCo parameters
print("[INFO] detecting '{}' tags...".format(args["type"]))

# get used aruco dictionary and create aruco detection parameters
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters()

# var for getting nearest aruco and wait for camera to activate
nearest = None
wetArucos = []
done = False
returning = False
inPlace = False

# vehicle = pixhawk.connectMyCopter()
# pixhawk.arm(vehicle)

# initialize the video stream and allow the camera sensor to warm up
print("[INFO] starting video stream...")
vs = cv2.VideoCapture(0)


marker_size = 0.3 # aruco marker sides measure 30cm
x_dist = None
y_dist = None
z_dist = None
time.sleep(2.0)

# loop over the frames from the video stream
while True:
	# grab the frame from the threaded video stream and resize it
	# to have a maximum width of 1000 pixels
	ret, frame = vs.read()
	# frame = imutils.resize(frame, width=1000)
	# frame = cv2.resize(frame, (64, 64))
	
	# detect ArUco markers in the input frame
	(corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
	
	# get center of frame and put a circle there
	(h, w) = frame.shape[:2] #w:image-width and h:image-height
	cv2.circle(frame, (w//2, h//2), 7, (255, 255, 255), -1) 
	center = (w//2, h//2)

    	# verify *at least* one ArUco marker was detected
	if len(corners) > 0 and not returning:
		# flatten the ArUco IDs list
		ids = ids.flatten()

		# aruco list for storing the ones in the frame and index for looping over the arus list
		arus = [Aruco(id) for id in ids]
		index = 0
		
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned
			# in top-left, top-right, bottom-right, and bottom-left
			# order)
			# contour = markerCorner[0]
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))
			

			# compute and draw the center (x, y)-coordinates of the
			# ArUco marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			arucoCenter = (cX, cY)
			cv2.circle(frame, arucoCenter, 4, (0, 0, 255), -1)


			# draw the bounding box of the ArUCo detection
			# colors are in BGR format
			if (markerID == 12) :
				color = (0, 255, 0)

			elif markerID in wetArucos:
				color = (255, 0, 0)
				
			elif (nearest is not None and nearest.id == markerID):
				color = (0, 0, 255)
				cv2.line(frame, center, arucoCenter, (255, 0, 255), 2)
				cv2.arrowedLine(frame, center, (center[0],arucoCenter[1]), (0, 165, 255), 2)
				cv2.arrowedLine(frame, center, (arucoCenter[0],center[1]), (0, 165, 0), 2)
				nearest.X_coordinate = arucoCenter[0]
				nearest.Y_coordinate = arucoCenter[1]

				#send location to pixhawk y el commnand pa que vaya
				# baje el drone

				# cv2.putText(frame, f"X Distance: {nearest.x_dist} meters, Y Distance: {nearest.y_dist} meters, Z distance: {nearest.z_dist}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 1)

            

			else:
				color = (88, 219, 255)
				# color = (255, 219, 88)
			cv2.line(frame, topLeft, topRight, color, 2)
			cv2.line(frame, topRight, bottomRight,color, 2)
			cv2.line(frame, bottomRight, bottomLeft, color, 2)
			cv2.line(frame, bottomLeft, topLeft, color, 2)

			aruco_area = cv2.contourArea(corners)
			
			rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(markerCorner, marker_size, camera_matrix, distortion_coeffs)
			if len(tvecs) > 0:
				# Extracting the X and Y distances
				arus[index].x_dist = tvecs[0][0][0]  # X component of the translation vector for the first marker
				arus[index].y_dist = tvecs[0][0][1]  # Y component of the translation vector for the first marker
				arus[index].z_dist = tvecs[0][0][2]  # Z component of the translation vector for the first marker
				print("X Distance: {:.2f} meters, Y Distance: {:.2f} meters".format(arus[index].x_dist, arus[index].y_dist))
			
			# pixel_cm_ratio = aruco_perimeter/(30*30)
			
			# curr_height = vehicle.location.global_relative_frame.alt

			# aqui pa mover el dron al sitio donde se calculo el aruco
			# if nearest is not None and nearest.id not in wetArucos:# and not inPlace:
			# 	# poner al pixhawk en guided mode
			# 	pixhawk.guided(vehicle)
			# 	# mover el pixhawk a la localizacion del aruco y bajarlo a 3 metros
			# 	pixhawk.go_to_location(nearest.x_dist, nearest.y_dist, 3, vehicle)
			# 	inPlace = True

				# apuntar la camara para abajo, suponiendo que llego bien o cerca al aruco

			#aqui pa mojar el aruco. check eso de la distancia al centro del frame, cuan cerca lo queremos
			if (nearest is not None) and (nearest.id not in wetArucos):
				#aqui se moja el aruco activando la bomba
				
				'''###### MOJAR COMO SEA#####'''

				nearest.wetStatus = True
				wetArucos.append(nearest.id)

				#sube el dron
				# pixhawk.change_height(vehicle, curr_height)
				# #set auto mode
				# pixhawk.auto(vehicle)


			# calculate the distance of every aruco to the center of the frame
			arus[index].distance_from_frame_center = (math.sqrt((arucoCenter[0]-center[0])**2 + (arucoCenter[1]-center[1])**2))
			# arus[index].distance_from_frame_center = np.linalg.norm(tvecs[0][0][0], tvecs[0][0][1])
			# print(arus[index].distance)
# 
			arus[index].area = aruco_area			
			index += 1
			
			if markerID in wetArucos:
				cv2.putText(frame, "WET", arucoCenter, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

			# draw the ArUco marker ID on the frame
			cv2.putText(frame, str(markerID),
				(topLeft[0], topLeft[1] - 15),
				cv2.FONT_HERSHEY_SIMPLEX,
				0.5, color, 2)
			
		# sort arucos by distance to the center of the frame
		# arus = sorted(arus, key=lambda aruco: aruco.distance)
		arus = sorted(arus, key=lambda aruco: aruco.area, reverse=True)

		# to select the nearest to the center: 
		#	1. set the aruco closest to the center of the frame as the nearest (ignoring our teams one)
		#	2. if our aruco is the only one in the frame, ignore it
		#	3. If our aruco is the nearest to the center and there is at least another one, set the next one as the nearest. 
		if (arus[0].id != 12 and (arus[0].id not in wetArucos)):
			nearest = arus[0]
		elif ((arus[0].id == 12 or arus[0].id in wetArucos) and len(arus)==1):
			nearest = None

		elif ((arus[1].id == 12 or arus[1].id in wetArucos) and (arus[0].id == 12 or arus[0].id in wetArucos) and len(arus)==2):
			nearest = None

		elif (arus[0].id == 12 or arus[0].id in wetArucos) and (arus[1].id != 12 or arus[1].id not in wetArucos) and len(arus)>=2:
			nearest = arus[1]
		else:
			nearest = arus[2]

		if nearest is not None:
			cv2.putText(frame, f"Nearest distance: {nearest.distance_from_frame_center}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			
		print(nearest)
		print(wetArucos)


	# if there is no arucos in frame, there is no nearest one and the array of arucos is emptied. 
	else:
		nearest = None
		arus = []
	 

	if len(wetArucos) == 2:
		cv2.putText(frame, "Mission accomplished. Returning home", (center[0]-250, center[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
		returning = True
		# start = time.time()
	if returning:
		#pixwhawk go home
  
	
	# if location del pixhawk == home:
		done = True
		# pixhawk.return_home(vehicle)
		# pixhawk.disarm(vehicle)
		# pixhawk.close(vehicle)
		# break

	# show the output frame
	cv2.imshow("Frame", frame)
	
	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break
	if done:# and time.time()-start > 10:
		time.sleep(10)
		break

	
	
# do a bit of cleanup
cv2.destroyAllWindows()
# vs.stop()
