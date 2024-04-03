import rospy
from sensor_msgs.msg import Image
import cv2
from src.IMX219CameraCalibration.gst_cam import camera

class Camera:
    def __init__(self):
        rospy.init_node('imx219_node', anonymous=True)
        self.publisher = rospy.Publisher('imx/images', Image, queue_size=10)

        # Initialize the OpenCV bridge
        self.width = 1640
        self.height = 1232
        camera = cv2.VideoCapture(camera(0, self.width, self.height))
        self.rate = rospy.Rate(24)

    def start(self):
            
            while not rospy.is_shutdown():
                # Capture a frame from the camera
                ret_0, frame_0 = camera.read()

                if not ret_0:
                    rospy.logerr('Failed to capture image')
                    return
                
                if frame_0 is not None:
                    # Convert the frame to a ROS Image message
                    image_msg = self.bridge.cv2_to_imgmsg(frame_0, "bgr8")

                    # Publish the image message to the camera topic
                    self.publisher.publish(image_msg)
                self.rate.sleep()

if __name__ == '__main__':
    try:
        camera = Camera()
        camera.capture_and_publish()
    except rospy.ROSInterruptException:
        pass