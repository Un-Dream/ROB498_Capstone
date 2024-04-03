#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from gst_cam import camera

class Camera:
    def __init__(self):
        rospy.init_node('imx219_node', anonymous=True)
        self.publisher = rospy.Publisher('/imx/images', Image, queue_size=10)

        # Initialize the OpenCV bridge
        self.width = 1640
        self.height = 1232
        self.camera = cv2.VideoCapture(camera(0, self.width, self.height))
        self.rate = rospy.Rate(1)
        self.bridge = CvBridge()

    def start(self):
            
        while not rospy.is_shutdown():
            # Capture a frame from the camera
            ret_0, frame_0 = self.camera.read()

            if not ret_0:
                rospy.logerr('Failed to capture image')
                return
            
            if frame_0 is not None:
                # rospy.loginfo('has image')


                #TODO add colour correction if we want
                #This is now publishing our images
                #Make a new node to sub to /imx/images, and positional stuff 
                #process using opencv code <<--- AMY
                


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