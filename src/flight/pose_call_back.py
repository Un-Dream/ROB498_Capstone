#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseArray

class PoseSub:
    def __init__(self):
        rospy.init_node('pose_array_subscriber', anonymous=True)

        self.rate = rospy.Rate(10)
        # self.pose_array_pub = rospy.Publisher('pose_array_topic', PoseArray, queue_size=10)
        # self.pose_array_sub = rospy.Subscriber('/pose_array_publisher/pose_array_topic', PoseArray, self.callback)

        # self.waypoints = []
    
    def pose_array_publisher(self):
        pose_array_msg = PoseArray()
        # Populate the PoseArray message with some example poses
        for i in range(5):
            pose = Pose()
            pose.position.x = i * 0.1
            pose.position.y = i * 0.1
            pose.position.z = i * 0.1
            pose.orientation.w = 1.0  # Identity quaternion
            pose_array_msg.poses.append(pose)
        # Publish the PoseArray message
        self.pose_array_pub.publish(pose_array_msg)
        # rate.sleep()

    def callback(self, data):
        rospy.loginfo(data)
        #extract the data
        #save into our waypoint queu
    
    def pose_array_subscriber(self):
        self.pose_array_sub = rospy.Subscriber('pose_array_topic', PoseArray, self.callback)

def main():
    posePS = PoseSub()
    while not rospy.is_shutdown():
        # posePS.pose_array_publisher()
        posePS.pose_array_subscriber()
        posePS.rate.sleep()

if __name__ == '__main__':
    try:
        main()

    except rospy.ROSInterruptException:
        pass