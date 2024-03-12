#!/usr/bin/env python3

import rospy
import mavros_msgs

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header



class Controller:
    def __init__(self):
        self.curr_position = None
        rospy.init_node('fly', anonymous = True)
        # create sub and pub
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.rate = rospy.Rate(50) #10 Hz
        
        self.state = State()
        rospy.wait_for_service("mavros/set_mode")              
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback) 

        if self.state.mode != "OFFBOARD":
            rospy.loginfo("switch offboard")
            self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")


        rospy.wait_for_service("mavros/cmd/arming")
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        while True:
            # rospy.loginfo("set pose")
            # position_msg = PoseStamped()
            # position_msg.header = Header()
            # position_msg.header.stamp = rospy.Time.now()
            # position_msg.pose.position.x = 0
            # position_msg.pose.position.y = 0
            # position_msg.pose.position.z = 0

            # self.pub.publish(position_msg)

            msg = CommandBoolRequest()
            msg.value = True
            response = arming_client.call(msg)
            if response.success:
                rospy.loginfo('success')
            else:
                rospy.loginfo(response)



        rospy.loginfo("finish setup")

    def position_callback(self, msg):
        # update current position
        self.curr_position = msg.pose.position

    def state_callback(self, state):
        self.state = state

    def set_position(self):

        
        
        rospy.loginfo("set pose")
        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = 0
        position_msg.pose.position.y = 0
        position_msg.pose.position.z = 5

        self.pub.publish(position_msg)
        self.rate.sleep()


if __name__ == "__main__":
    try: 
        controller = Controller()
        # while not rospy.is_shutdown():
        #     # rospy.loginfo(controller.curr_position)

        #     controller.set_position()

    except rospy.ROSInterruptException:
        pass