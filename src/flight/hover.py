#!/usr/bin/env python3

# Code for flight exercise 2
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
from std_srvs.srv import Empty, EmptyResponse


class Controller:
    def __init__(self):
        
        # rospy.init_node('hover', anonymous = True)
        node_name = 'rob498_drone_05'
        rospy.init_node(node_name) 
        self.rate = rospy.Rate(60)
        # mavros publishers
        self.mavros_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.mavros_odom = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size = 10)

        # camera data
        self.camera_odom = rospy.Subscriber('/camera/odom/sample', Odometry, self.camera_callback)

        # TODO -> vicon data feeds into what?
        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', PoseStamped, self.vicon_callback)

        # TODO -> have a trigger for camera pose on camera vs vicon
        # Internal variables
        self.vicon_pose = None
        self.camera_pose = None
        self.camera_data_full = None
        self.curr_position = None
        self.control_mode = "IDLE"
        

        self.state = State()
        rospy.loginfo('end')
        rospy.wait_for_service("mavros/set_mode")              
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.state_sub = rospy.Subscriber("/mavros/state", State, self.state_callback) 
        rospy.loginfo('off')
        while self.state.mode != "OFFBOARD":
            rospy.loginfo("switch offboard")
            offboard_req = SetModeRequest()
            offboard_req.custom_mode = "OFFBOARD"
            self.set_mode_client(offboard_req)

        rospy.wait_for_service("mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.if_armed = False

        # initialize start pose for drone
        while (self.camera_pose is None):
            rospy.loginfo("Waiting for init pose")
            pass
        self.start_pose  = self.camera_pose
        self.goal_point = [0, 0, self.start_pose.z+0.5]

        rospy.loginfo("finish startup")


        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)

    def state_callback(self, state):
        self.state = state

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


    def vicon_callback(self, msg):
        # update current position
        self.vicon_pose = msg.pose.position

    def set_position(self, position):
        if self.if_armed == False:
            response = self.arming_client.call(True)
            if response.success:
                rospy.loginfo('success')
                self.if_armed = True
            else:
                rospy.loginfo(response)

        # self.rate.sleep()
        
        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = position[0]
        position_msg.pose.position.y = position[1]
        position_msg.pose.position.z = position[2]
        self.mavros_pub.publish(position_msg)    


    def tolerance_error(self, goal_point):
        # error = ((self.camera_pose.x - goal_point[0]) ** 2 + (self.camera_pose.y - goal_point[1]) **2 + (self.camera_pose.z - goal_point[2])**2)**0.5
        error = ((self.camera_pose.z - goal_point[2])**2)**0.5
        return error


    def set_position(self, position):
        if self.if_armed == False:
            response = self.arming_client.call(True)
            if response.success:
                rospy.loginfo('success')
                self.if_armed = True
            else:
                rospy.loginfo(response)        
        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = position[0]
        position_msg.pose.position.y = position[1]
        position_msg.pose.position.z = position[2]
        self.mavros_pub.publish(position_msg)

    # Callback handlers
    def handle_launch():
        print('Launch Requested. Your drone should take off.')
        # FLY TO GOAL POINT
        self.control_mode = "FLY"

    def handle_test():
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        # # HOVER
        self.control_mode = "HOVER"

    def handle_land():
        print('Land Requested. Your drone should land.')
        # LAND AT START POSE
        self.control_mode = "LAND"

    def handle_abort():
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        # LAND AT CURRENT POSITION
        self.control_mode = "ABORT"

    # Service callbacks
    def callback_launch(self, request):
        self.handle_launch()
        return EmptyResponse()

    def callback_test(self, request):
        self.handle_test()
        return EmptyResponse()

    def callback_land(self, request):
        self.handle_land()
        return EmptyResponse()

    def callback_abort(self, request):
        self.handle_abort()
        return EmptyResponse()

    def flight_exercise_2_auto(self):
        self.control_mode = "FLY" # start in this mode
        start_time = None
        error = 1000
        base_error = 1000
        
        while not rospy.is_shutdown():
            if self.control_mode == "FLY":
                rospy.loginfo("Flying towards point")
                self.set_position(self.goal_point)
                error = self.tolerance_error(self.goal_point)
                self.rate.sleep()
                if error < 0.05:
                    self.control_mode = "HOVER"
                    start_time = time.time()
            elif self.control_mode == "HOVER":
                rospy.loginfo("hover")
                self.set_position(self.goal_point)
                self.rate.sleep()
                if time.time() - start_time > 10:
                    self.control_mode = "RETURN"
            elif self.control_mode == "RETURN":
                rospy.loginfo("fly home")
                self.set_position([self.start_pose.x, self.start_pose.y, self.start_pose.z])
                base_error = self.tolerance_error([self.start_pose.x, self.start_pose.y, self.start_pose.z])
                self.rate.sleep()
                if base_error < 0.05:
                    self.control_mode = "END"
            elif self.control_mode == "END":
                rospy.loginfo("end")
                rospy.spin()

    def flight_exercise_2_test(self):
        self.control_mode = "IDLE" # start in this mode
        start_time = None
        base_error = 1000

        # SERVICE BASED TRIGGER
        while not rospy.is_shutdown():
            if self.control_mode == "FLY":
                rospy.loginfo("Flying towards point")
                self.set_position(self.goal_point)
                error = self.tolerance_error(self.goal_point)
                self.rate.sleep()
            elif self.control_mode == "HOVER":
                rospy.loginfo("hover")
                self.set_position(self.goal_point)
                self.rate.sleep()
            elif self.control_mode == "LAND":
                rospy.loginfo("fly home")
                self.set_position([self.start_pose.x, self.start_pose.y, self.start_pose.z])
                base_error = self.tolerance_error([self.start_pose.x, self.start_pose.y, self.start_pose.z])
                self.rate.sleep()
                if base_error < 0.05:
                    self.control_mode = "END"
            elif self.control_mode == "ABORT":
                rospy.loginfo("abort")
                self.set_position([self.camera_pose.x, self.camera_pose.y, self.start_pose.z])
                base_error = self.tolerance_error([self.camera_pose.x, self.camera_pose.y, self.start_pose.z])
                self.rate.sleep()
                if base_error < 0.05:
                    self.control_mode = "END"
            elif self.control_mode == "END":
                rospy.loginfo("end")
                rospy.spin()
            else:
                rospy.loginfo("waiting for launch")
        



if __name__  == "__main__":
    try: 
        controller = Controller()
        controller.flight_exercise_2_auto()
        

    except rospy.ROSInterruptException:
            pass