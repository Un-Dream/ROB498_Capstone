#!/usr/bin/env python

import rospy
from mavros_msgs.msg import SetMode, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from math import cos, sin

class HoverController:
    def __init__(self):
        rospy.init_node('hover_controller_node', anonymous=True)
        self.rate = rospy.Rate(10)  # 10Hz
        self.hover_altitude = 2.0  # Hover altitude in meters
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.current_position_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.current_position = PoseStamped()
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    def position_callback(self, msg):
        self.current_position = msg

    def arm_disarm(self, arm):
        try:
            self.arm(arm)
            rospy.loginfo("Vehicle armed: {}".format(arm))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def set_mode_flight(self, mode):
        try:
            self.set_mode(0, mode)
            rospy.loginfo("Flight mode set to: {}".format(mode))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {}".format(e))

    def hover(self):
        self.arm_disarm(True)
        self.set_mode_flight("OFFBOARD")

        while not rospy.is_shutdown():
            setpoint_msg = PoseStamped()
            setpoint_msg.header = Header()
            setpoint_msg.header.stamp = rospy.Time.now()
            setpoint_msg.pose.position.x = self.current_position.pose.position.x
            setpoint_msg.pose.position.y = self.current_position.pose.position.y
            setpoint_msg.pose.position.z = self.hover_altitude

            self.setpoint_pub.publish(setpoint_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = HoverController()
        controller.hover()
    except rospy.ROSInterruptException:
        pass