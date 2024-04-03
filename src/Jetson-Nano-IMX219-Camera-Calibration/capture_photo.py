#!/usr/bin/env python3

import cv2
import time
from gst_cam import camera

# w, h = 3280, 2464
w, h = 1640, 1232
cap_0 = cv2.VideoCapture(camera(0, w, h))

time.sleep(3)

i = 0
while True :
	ret_0, frame_0 = cap_0.read()

	if not ret_0 :
		break

	cv2.imshow("frame 0", cv2.resize(frame_0, (0,0), fx=0.25, fy=0.25))
	
	key = cv2.waitKey(10)
	if key == ord("q"):
		break

	if key == ord("s"):
		print(i)
		print("saved image %d" % i)
		cv2.imwrite("/home/jetson/ROB498_FlightExercises/src/Jetson-Nano-IMX219-Camera-Calibration-OpenCV/capture_lowResolution/cam_0_%d.jpg" % i, frame_0)
		i += 1

	if i == 10 :
		break

cv2.destroyAllWindows()
cap_0.release()
