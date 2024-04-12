#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from gst_cam import camera
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray, Point
from nav_msgs.msg import Odometry

from apriltag import Detector


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
        img = cv2.resize(self.img, (0,0), fx=0.25, fy=0.25)

        h, w, c = img.shape
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 0, (w,h))


    def mavros_callback(self,msg):
        self.curr_position = msg.pose.pose.position


    def start(self):
        while not rospy.is_shutdown():
            # Capture a frame from the camera
            ret_0, frame_0 = self.camera.read()
            # print(frame_0)
            if not ret_0:
                rospy.logerr('Failed to capture image')
                return
            
            if frame_0 is not None:
                rospy.loginfo('has image')
                img = cv2.resize(img, (0,0), fx=0.25, fy=0.25)
                h, w, c = img.shape
                self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 0, (w,h))

                dst = cv2.undistort(frame_0, self.mtx, self.dist, None, self.newcameramtx)
                print(dst.shape)
                # crop the image
                x, y, w, h = self.roi
                dst = dst[y:y+h, x:x+w]
                
                #TODO add colour correction if we want
                #This is now publishing our images
                #Make a new node to sub to /imx/images, and positional stuff 
                #process using opencv code <<--- AMY


                # color space 
                hsvFrame = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV) 
            
                # Set range for red color and  
                # define mask 
                red_lower = np.array([136, 87, 111], np.uint8) 
                red_upper = np.array([180, 255, 255], np.uint8) 
                red_mask = cv2.inRange(hsvFrame, red_lower, red_upper) 
                    # Creating contour to track red color 
                contours, hierarchy = cv2.findContours(red_mask, 
                                                    cv2.RETR_TREE, 
                                                    cv2.CHAIN_APPROX_SIMPLE) 
                
                colour_list = []

                #TODO replace image path / matric
                image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

                # Create AprilTag detector
                detector = Detector(families='tag36h11')

                # Detect tags in the image
                detections = detector.detect(image)

                # Filter H11 tags
                h11_tags = [d for d in detections if d.tag_id == 11]


                if h11_tags:
                    print("H11 AprilTag detected!")
                    for tag in h11_tags:
                        print("Tag ID:", tag.tag_id)
                        print("Center:", tag.center)
                        print("Corners:", tag.corners)
                else:
                    print("H11 AprilTag not detected.")
                
                # for pic, contour in enumerate(contours): 
                #     area = cv2.contourArea(contour) 
                #     if(area > 300): 

                #         #find the center of the contour
                #         M = cv2.moments(contour)
                #         cX = int(M["m10"] / M["m00"])
                #         cY = int(M["m01"] / M["m00"])
                #         # cv2.circle(imageFrame, (cX, cY), 5, (255, 255, 255), -1)

                #         #backwards projection
                #         #get the depth of the pixel
                #         z = self.curr_position.z
                #         #get the x and y of the pixel
                #         x = cX
                #         y = cY

                #         #camera intrinsics
                #         # K = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                #         K = self.mtx

                #         """
                #         K = np.array([[focal_length_x, 0, principal_point_x],
                #         [0, focal_length_y, principal_point_y],
                #         [0, 0, 1]])
                #         """
                #         K_inv = np.linalg.inv(K)

                #         #get the pixel in camera frame
                #         pixel = np.array([x, y, 1])
                #         point_camera = np.dot(K_inv, pixel) * z

                #         #make into pose type
                #         pose_camera = Point()
                #         pose_camera.x = point_camera[0]
                #         pose_camera.y = point_camera[1]
                #         pose_camera.z = point_camera[2]


                #         #get the pixel in world frame
                #         #add them together
                #         point_world = Point()
                #         point_world.x = self.curr_position.x + pose_camera.x
                #         point_world.y = self.curr_position.y + pose_camera.y
                #         point_world.z = self.curr_position.z + pose_camera.z

                #         #publish the point
                #         rospy.loginfo(point_world)
                #         # self.publisher_detection.publish(point_world)

                            
                # Convert the frame to a ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(dst, "bgr8")

                #save image
                cv2.imwrite('image.png', dst)


                # Publish the image message to the camera topic
                self.publisher.publish(image_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        camera = Camera()
        camera.start()
    except rospy.ROSInterruptException:
        pass