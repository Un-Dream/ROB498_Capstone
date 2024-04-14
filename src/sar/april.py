#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
import cv2
from cv_bridge import CvBridge
from gst_cam import camera_
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray, Point
from nav_msgs.msg import Odometry

# from apriltag import Detector, DetectorOptions

from apriltag_ros.msg import AprilTagDetectionArray


print(cv2.__version__)


class Camera:
    def __init__(self):
        rospy.init_node('imx219_node', anonymous=True)
        self.publisher = rospy.Publisher('/camera_rect/image_rect', Image, queue_size=10)
        self.publisher_detection = rospy.Publisher('/detection/pose', Point, queue_size=10)
        self.mavros_pose = rospy.Subscriber('/mavros/odometry/in', Odometry, self.mavros_callback)
        self.tag_subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.camera_info_publisher = rospy.Publisher('/camera_rect/camera_info', CameraInfo, queue_size=10)
        
        self.curr_position = Point()
        self.curr_position.x = 0
        self.curr_position.y = 0
        self.curr_position.z = 0


        # Initialize the OpenCV bridge
        self.width = 1280 #1640
        self.height =  720 #1232
        # self.camera = cv2.VideoCapture(camera(0, self.width, self.height))

        self.mtx = np.array([[1.90141428e+02, 0.00000000e+00, 1.93371637e+02],
                    [0.00000000e+00, 1.90074752e+02, 1.36530788e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.dist = np.array([[-0.33485375, 0.14237009, 0.00069925, 0.00196279, -0.03062868]])
        self.rate = rospy.Rate(60)
        self.bridge = CvBridge()

        self.camera = cv2.VideoCapture(camera_(0, self.width, self.height), cv2.CAP_GSTREAMER)

        print(self.camera)

        __, img = self.camera.read()
        while img is None :
            # rospy.loginfo("h")
            # self.camera = cv2.VideoCapture(camera(0, self.width, self.height))
            __, img = self.camera.read()
        img = cv2.resize(img, (0,0), fx=0.25, fy=0.25)

        h, w, c = img.shape
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 0, (w,h))
        
        # print(self.roi)

        self.camera_info_msg = CameraInfo()

        # Fill in the CameraInfo message with your values
        self.camera_info_msg.header.stamp = rospy.Time.now()
        self.camera_info_msg.header.frame_id = "camera_optical_frame"
        self.camera_info_msg.height = h  # Replace with your camera's height
        self.camera_info_msg.width = w   # Replace with your camera's width
        self.camera_info_msg.distortion_model = "plumb_bob"

        # Distortion parameters (D)
        self.camera_info_msg.D = self.dist[0].tolist()  # Replace with your camera's distortion parameters

        # Intrinsic camera matrix (K)
        self.camera_info_msg.K = self.newcameramtx.flatten().tolist()  # Replace fx, fy, cx, cy with your camera's intrinsics

        # Rectification matrix (R)
        self.camera_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Assuming no rotation, replace if needed

        # Projection/camera matrix (P)
        # self.camera_info_msg.P = [fx, 0, cx, Tx, 0, fy, cy, Ty, 0, 0, 1, 0]  # Replace fx, fy, cx, cy, Tx, Ty with your camera's projection parameters
        # Projection/camera matrix (P)
        fx, fy, cx, cy = self.mtx[0, 0], self.mtx[1, 1], self.mtx[0, 2], self.mtx[1, 2]
        self.camera_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]  # Set Tx and Ty to 0
        
        self.ROI = RegionOfInterest()
        self.ROI.x_offset = self.roi[0]
        self.ROI.y_offset = self.roi[1]
        self.ROI.height = self.roi[3]
        self.ROI.width = self.roi[2]

        
        # Operational parameters
        self.camera_info_msg.binning_x = 0
        self.camera_info_msg.binning_y = 0
        self.camera_info_msg.roi = self.ROI

        self.Tx = np.array([[1,0,0,1],
                           [0,1,0,0],
                           [0,0,1,0],
                           [0,0,0,1]])
        

        self.R = np.array([[-1,0,0,0],
                           [0,0,1,0],
                           [0,1,0,0],
                           [0,0,0,1]])
        
        self.T = np.dot(self.Tx, self.R)

    def mavros_callback(self,msg):
        self.curr_position = msg.pose.pose.position

    def tag_callback(self, msg):

        array = msg.detections


        #TODO
        # put to world frame msg

        for i in array:
            # #relative to camera transform to world
            # transform monocular to stereo
            # translation
            # rotation of yaw of 90

            
            pose = i.pose.pose.pose.position

            p = np.array([pose.x, pose.y, pose.z, 1])

            world_pose = np.matmul(self.T, p.T)


            # rospy.loginfo(world_pose)
        # rospy.loginfo(msg)



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
                dst_gray_pub = self.bridge.cv2_to_imgmsg(dst_gray)
                dst_gray_pub.header.stamp = rospy.Time.now()
                self.publisher.publish(dst_gray_pub)

                # self.camera_info_msg.header.stamp = rospy.Time.now()
                self.camera_info_msg.header.stamp =  dst_gray_pub.header.stamp

                self.camera_info_publisher.publish(self.camera_info_msg)

                # rospy.loginfo("image")


                # print(dst[int(dst.shape[0]/2), int(dst.shape[1]/2),:])

                # cv2.imwrite('image.png', dst)

                # cv2.imshow('Calibration Result', dst_gray)
                # key = cv2.waitKey(10)
                # if key == ord("q"):
                #     self.rate.sleep()
                #     break

            self.rate.sleep()

if __name__ == '__main__':
    try:
        camera = Camera()
        camera.start()
    except rospy.ROSInterruptException:
        pass