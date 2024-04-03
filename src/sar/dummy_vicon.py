#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def publish_transform():
    rospy.init_node('transform_publisher_node', anonymous=True)
    broadcaster = tf2_ros.TransformBroadcaster()
    vicon_pub = rospy.Publisher('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, queue_size = 10)

    rate = rospy.Rate(10)  # 10 Hz

    x = 0
    y = 0
    z = 0

    while not rospy.is_shutdown():
        # Create a TransformStamped message
        transform_stamped = TransformStamped()

        # Fill in the header
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "parent_frame"
        transform_stamped.child_frame_id = "child_frame"

        # Fill in the transformation
        transform_stamped.transform.translation.x = min(x,10)  # Example translation in X
        transform_stamped.transform.translation.y = min(y,10)  # Example translation in Y
        transform_stamped.transform.translation.z = min(z, 10)  # Example translation in Z
        transform_stamped.transform.rotation.x = 0.0  # Example rotation quaternion X
        transform_stamped.transform.rotation.y = 0.0  # Example rotation quaternion Y
        transform_stamped.transform.rotation.z = 0.0  # Example rotation quaternion Z
        transform_stamped.transform.rotation.w = 1.0  # Example rotation quaternion W

        # Publish the transform
        broadcaster.sendTransform(transform_stamped)
        vicon_pub.publish(transform_stamped)

        x+=1
        y+=1
        z+=1


        rate.sleep()

if __name__ == '__main__':
    try:
        publish_transform()
    except rospy.ROSInterruptException:
        pass