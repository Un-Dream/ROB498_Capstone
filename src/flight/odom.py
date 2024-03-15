#!/usr/bin/env python3

# Publish data to odom topic

import math
import tf
import rospy
import mavros_msgs
import time

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, SetModeRequest
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class Odom:

    def __init__(self):
        rospy.init_node('odom_node', anonymous = True)

        # send to mavros odom out
        self.mavros_odom = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size = 10)

        # camera data
        self.camera_odom = rospy.Subscriber('/camera/odom/sample', Odometry, self.camera_callback)

        # TODO -> vicon data feeds into what?
        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', PoseStamped, self.vicon_callback)
        

        # TODO -> have a trigger for camera pose on camera vs vicon
        self.camera_pose = None
        self.camera_data_full = None

    def camera_callback(self, msg):
        self.camera_data_full = msg
        self.camera_pose = msg.pose.pose.position
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = msg.pose.pose.position.x
        odom_msg.pose.pose.position.y = msg.pose.pose.position.y
        odom_msg.pose.pose.position.z = msg.pose.pose.position.z
        odom_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
        odom_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y
        odom_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
        odom_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.mavros_odom.publish(odom_msg)


if __name__ == __main__:
    odom()