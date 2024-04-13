#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from gst_cam import camera
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray, Point
from nav_msgs.msg import Odometry

from apriltag import Detector, DetectorOptions


class Camera:
    def __init__(self):
        rospy.init_node('imx219_node', anonymous=True)
        self.publisher = rospy.Publisher('/imx/images', Image, queue_size=10)
        self.publisher_detection = rospy.Publisher('/detection/pose', Point, queue_size=10)
        self.mavros_pose = rospy.Subscriber('/mavros/odometry/in', Odometry, self.mavros_callback)
        self.curr_position = Point()
        self.curr_position.x = 0
        self.curr_position.y = 0
        self.curr_position.z = 0


        # Initialize the OpenCV bridge
        self.width = 1640
        self.height = 1232
        # self.camera = cv2.VideoCapture(camera(0, self.width, self.height))

        self.mtx = np.array([[1.90141428e+02, 0.00000000e+00, 1.93371637e+02],
                    [0.00000000e+00, 1.90074752e+02, 1.36530788e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.dist = np.array([[-0.33485375, 0.14237009, 0.00069925, 0.00196279, -0.03062868]])
        self.rate = rospy.Rate(60)
        self.bridge = CvBridge()

        # img = None

        # while img is not None:
        #     __, img = self.camera.read()

        #     if img is None :
        #         continue
        #     img = cv2.resize(img, (0,0), fx=0.25, fy=0.25)
        #     h, w, c = img.shape

        #     self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 0, (w,h))

        # if img is None:
        self.camera = cv2.VideoCapture(camera(0, self.width, self.height))
        __, img = self.camera.read()
        while img is None :
            self.camera = cv2.VideoCapture(camera(0, self.width, self.height))
            __, img = self.camera.read()
        img = cv2.resize(img, (0,0), fx=0.25, fy=0.25)

        h, w, c = img.shape
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 0, (w,h))


    def mavros_callback(self,msg):
        self.curr_position = msg.pose.pose.position


    def start(self):
        while not rospy.is_shutdown():
            ret_0, frame_0 = self.camera.read()
            if not ret_0:
                rospy.logerr('Failed to capture image')
                return
            
            if frame_0 is not None:
                # rospy.loginfo('has image')
                frame_0 = cv2.resize(frame_0, (0,0), fx=0.25, fy=0.25)
                h, w, c = frame_0.shape
                self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 0, (w,h))

                dst = cv2.undistort(frame_0, self.mtx, self.dist, None, self.newcameramtx)
                x, y, w, h = self.roi
                dst = dst[y:y+h, x:x+w]

                #Change to grayscale
                dst_gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)

                # Create AprilTag detector
                options = DetectorOptions(families='tag36h11')
                detector = Detector(options)

                # Detect tags in the image
                detections = detector.detect(dst_gray)

                # Filter H11 tags, detect id 11 only
                h11_tags = [d for d in detections if d.tag_id == 11]

                if h11_tags:
                    print("H11 AprilTag detected!")
                    for tag in h11_tags:
                        # print("Tag ID:", tag.tag_id)
                        # print("Center:", tag.center)
                        # print("Corners:", tag.corners)
                        # M, init_error, final_error = detector.detection_pose(tag, self.mtx)
                        # print(M)
                        # TODO FIND POSITION
                        # HAVE A NODE TO LISTEN TO POSITION
                        # NEED TO PRINT POSITION
                        imagePoints = tag.corners.reshape(1,4,2)
                        tag_size = 165

                        ob_pt1 = [-tag_size/2, -tag_size/2, 0.0]
                        ob_pt2 = [ tag_size/2, -tag_size/2, 0.0]
                        ob_pt3 = [ tag_size/2,  tag_size/2, 0.0]
                        ob_pt4 = [-tag_size/2,  tag_size/2, 0.0]
                        ob_pts = ob_pt1 + ob_pt2 + ob_pt3 + ob_pt4
                        object_pts = np.array(ob_pts).reshape(4,3)

                        opoints = np.array([
                            -1, -1, 0,
                            1, -1, 0,
                            1,  1, 0,
                            -1,  1, 0,
                            -1, -1, -2*1,
                            1, -1, -2*1,
                            1,  1, -2*1,
                            -1,  1, -2*1,
                        ]).reshape(-1, 1, 3) * 0.5*tag_size

                        # mtx - the camera calibration's intrinsics
                        good, prvecs, ptvecs = cv2.solvePnP(object_pts, imagePoints, self.mtx, self.dist, flags=cv2.SOLVEPNP_ITERATIVE)
                        imgpts, jac = cv2.projectPoints(opoints, prvecs, ptvecs, self.mtx, self.dist)

                        # Draws the edges of the pose onto the image
                        draw_boxes(img, imgpts, edges)




                else:
                    print("H11 AprilTag not detected.")

                # print(dst[int(dst.shape[0]/2), int(dst.shape[1]/2),:])

                cv2.imshow('Calibration Result', dst_gray)
                # cv2.imwrite('image.png', dst)
                key = cv2.waitKey(10)
                if key == ord("q"):
                    self.rate.sleep()
                    break

            # self.rate.sleep()

if __name__ == '__main__':
    try:
        camera = Camera()
        camera.start()
    except rospy.ROSInterruptException:
        pass