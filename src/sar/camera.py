#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from gst_cam import camera
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray
from nav_msgs.msg import Odometry


class Camera:
    def __init__(self):
        rospy.init_node('imx219_node', anonymous=True)
        self.publisher = rospy.Publisher('/imx/images', Image, queue_size=10)
        self.publisher_detection = rospy.Publisher('/detection/pose', Pose, queue_size=10)
        self.mavros_pose = rospy.Subscriber('/mavros/odometry/in', Odometry, self.mavros_callback)


        # Initialize the OpenCV bridge
        self.width = 1640
        self.height = 1232
        self.camera = cv2.VideoCapture(camera(0, self.width, self.height))

        self.mtx = np.array([[1.90141428e+02, 0.00000000e+00, 1.93371637e+02],
                    [0.00000000e+00, 1.90074752e+02, 1.36530788e+02],
                    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.dist = np.array([[-0.33485375, 0.14237009, 0.00069925, 0.00196279, -0.03062868]])
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, (self.width,self.height), 0,  (self.width,self.height))


        

    def mavros_callback(self,msg):
        self.curr_position = msg.pose.pose.position

    def start(self):
            
        while not rospy.is_shutdown():
            # Capture a frame from the camera
            ret_0, frame_0 = self.camera.read()

            if not ret_0:
                rospy.logerr('Failed to capture image')
                return
            
            if frame_0 is not None:
                # rospy.loginfo('has image')
                dst = cv2.undistort(frame_0, self.mtx, self.dist, None, self.newcameramtx)
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
                
                for pic, contour in enumerate(contours): 
                    area = cv2.contourArea(contour) 
                    if(area > 300): 
   
                        #find the center of the contour
                        M = cv2.moments(contour)
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        # cv2.circle(imageFrame, (cX, cY), 5, (255, 255, 255), -1)

                        #backwards projection
                        #get the depth of the pixel
                        z = self.curr_position.z
                        #get the x and y of the pixel
                        x = cX
                        y = cY

                        #camera intrinsics
                        # K = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
                        K = self.mtx

                        """
                        K = np.array([[focal_length_x, 0, principal_point_x],
                        [0, focal_length_y, principal_point_y],
                        [0, 0, 1]])
                        """
                        K_inv = np.linalg.inv(K)

                        #get the pixel in camera frame
                        pixel = np.array([x, y, 1])
                        point_camera = np.dot(K_inv, pixel) * z

                        #get the pixel in world frame
                        point_world = self.curr_position + point_camera

                        #publish the point

                        self.publisher_detection.publish(point_world)


                            


                # Convert the frame to a ROS Image message
                image_msg = self.bridge.cv2_to_imgmsg(frame_0, "bgr8")

                # Publish the image message to the camera topic
                self.publisher.publish(image_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        camera = Camera()
        camera.start()
    except rospy.ROSInterruptException:
        pass