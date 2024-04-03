#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseArray

def pose_array_publisher():
    # Initialize the ROS node
    rospy.init_node('pose_array_publisher', anonymous=True)

    # Create a publisher for the PoseArray topic
    pose_array_pub = rospy.Publisher('/comm/dummy_waypoints', PoseArray, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        
        # Create a PoseArray message
        pose_array_msg = PoseArray()

        # Populate the PoseArray message with some example poses
        for i in range(3):
            pose = Pose()
            if i == 0:
                pose.position.x = 0.5
                pose.position.y = 0
                pose.position.z = 0
                pose.orientation.w = 1.0  # Identity quaternion
            elif i==1:
                pose.position.x = 0
                pose.position.y = 0.5
                pose.position.z = 0
                pose.orientation.w = 1.0  # Identity quaternion
            elif i==2:
                pose.position.x = 0
                pose.position.y = 0
                pose.position.z = 0.5
                pose.orientation.w = 1.0  # Identity quaternion
            pose_array_msg.poses.append(pose)

        # Publish the PoseArray message
        pose_array_pub.publish(pose_array_msg)
        # rospy.loginfo(pose_array_msg)
        # Sleep according to the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_array_publisher()
    except rospy.ROSInterruptException:
        pass
