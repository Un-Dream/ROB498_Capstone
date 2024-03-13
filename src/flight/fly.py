#!/usr/bin/env python3

import math

import rospy
import mavros_msgs

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, SetModeRequest
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header



class Controller:
    def __init__(self):
        self.curr_position = None
        rospy.init_node('fly', anonymous = True)
        # create sub and pub
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.odompub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size = 10)
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.rate = rospy.Rate(60) #10 Hz
        
        self.state = State()
        rospy.wait_for_service("mavros/set_mode")              
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback) 

        while self.state.mode != "OFFBOARD":
            rospy.loginfo("switch offboard")
            offboard_req = SetModeRequest()
            offboard_req.custom_mode = "OFFBOARD"
            self.set_mode_client(offboard_req)


        rospy.wait_for_service("mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.if_armed = False

        rospy.loginfo("arm")

        self.rate.sleep()

        self.start_pose  = self.curr_position
        rospy.loginfo(self.start_pose)
        rospy.loginfo('here')

    def position_callback(self, msg):
        # update current position
        self.curr_position = msg.pose.position

    def state_callback(self, state):
        self.state = state

    def fly_to_base(self):
        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = self.start_pose.x
        position_msg.pose.position.y = self.start_pose.y
        position_msg.pose.position.z = self.start_pose.z
        self.pub.publish(position_msg)


    def set_position(self, position):
        if self.if_armed == False:
            response = self.arming_client.call(True)
            if response.success:
                rospy.loginfo('success')
                self.if_armed = True
            else:
                rospy.loginfo(response)

        self.rate.sleep()
        
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation.x = 0
        odom_msg.pose.pose.orientation.y = 0
        odom_msg.pose.pose.orientation.z = 0
        odom_msg.pose.pose.orientation.w = 1
        self.odompub.publish(odom_msg)

        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = position[0]
        position_msg.pose.position.y = position[1]
        position_msg.pose.position.z = position[2]
        self.pub.publish(position_msg)

    def tolerance_error(self, goal_point):
        error = ((self.curr_position.x - goal_point[0]) ** 2 + (self.curr_position.y - goal_point[1]) **2 + (self.curr_position.z - goal_point[2])**2)**0.5
        return error

            


if __name__ == "__main__":
    try: 
        controller = Controller()
        goal_point = [0, 0, controller.start_pose.z+0.75]
        
        error = 1000
        base_error = 1000
        while not rospy.is_shutdown():
            rospy.loginfo("Flying towards point")
            while error > 0.5:
                # rospy.loginfo("Flying towards point")
                rospy.loginfo("Current pose: %s", controller.curr_position)
                controller.set_position(goal_point)
                error = controller.tolerance_error(goal_point)
            
            time = 0
            init_t = rospy.Time.now()
            rospy.loginfo("hover")
            while time < 20000:
                time = time + 1
                rospy.loginfo("Current pose: %s", controller.curr_position)
                controller.set_position(goal_point)
                # rospy.loginfo("Current pose: %s", controller.curr_position)

            rospy.loginfo("fly home")

            while base_error > 0.5:
                controller.fly_to_base()
                base_error = controller.tolerance_error([controller.start_pose.x, controller.start_pose.y, controller.start_pose.z])
                rospy.loginfo("Current pose: %s", controller.curr_position)

            rospy.loginfo('done')
            



            

    except rospy.ROSInterruptException:
        pass