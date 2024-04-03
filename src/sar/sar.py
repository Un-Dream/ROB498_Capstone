#!/usr/bin/env python3

import math
import tf
import rospy
import mavros_msgs
import time
import numpy as np

from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, SetModeRequest
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse


class Controller:
    def __init__(self):
        rospy.loginfo("start")
        # rospy.init_node('hover', anonymous = True)
        node_name = 'rob498_drone_05'
        rospy.init_node(node_name) 
        self.rate = rospy.Rate(60)
        # mavros publishers
        self.mavros_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        # self.mavros_odom = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size = 10)
        self.mavros_pose = rospy.Subscriber('/mavros/odometry/in', Odometry, self.mavros_callback)

        ### Subscriber Banks
        # camera data - dummy don't actually use this use mavros odom
        self.camera_odom = rospy.Subscriber('/camera/odom/sample', Odometry, self.camera_callback)        
        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.vicon_callback)

        # waypoint subscribe and variables
        # TODO double check this
        self.WAYPOINTS_RECEIVED = False
        self.waypoints = None
        # self.waypoints_sub = rospy.Subscriber('/start_pub_wpts_c3_05', PoseArray, self.callback_waypoints)
        self.waypoints_sub = rospy.Subscriber('/comm/waypoints', PoseArray, self.callback_waypoints)
        while self.waypoints is None:
            rospy.loginfo('waiting for waypoints')
            pass

        self.vicon_start_x = 0
        self.vicon_start_y = 0 
        self.vicon_start_z = 0

        # initialize the waypoint queue once - this is ok
        # self.waypoint_queue = self.waypoints #self.jiggle_generator(self.waypoints)
        self.waypoint_queue = self.jiggle_generator(self.waypoints)
        self.waypoint_curr = 0




    
        # Internal variables
        self.vicon_pose = None
        self.curr_position = None # use Mavros/odom/in
        self.control_mode = "IDLE"


        # Set mode to offboard for flight
        self.state = State()
        rospy.loginfo('set mode')
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
        while (self.curr_position is None):
            rospy.loginfo("Waiting for init pose")
            pass
        self.start_pose  = None
        self.goal_point = None
        self.vicon_start_z = 0

        rospy.loginfo("finish startup")

        self.srv_launch = rospy.Service(node_name + '/comm/launch', Empty, self.callback_launch)
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)
        self.srv_land = rospy.Service(node_name + '/comm/land', Empty, self.callback_land)
        self.srv_abort = rospy.Service(node_name + '/comm/abort', Empty, self.callback_abort)


    def mavros_callback(self,msg):
        self.curr_position = msg.pose.pose.position
        # rospy.loginfo(self.curr_position)

    def camera_callback(self, msg):
        # self.camera_data_full = msg
        # self.camera_pose = msg.pose.pose.position
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
        # self.mavros_odom.publish(odom_msg)

    def state_callback(self, state):
        self.state = state

    def vicon_callback(self, msg):
        # update current position
        try:
            self.vicon_pose = msg.transform
        except: 
            rospy.loginfo('vicon callback error - no data')

    def tolerance_error(self, goal_point):
        error = ((self.curr_position.x - goal_point[0]) ** 2 + (self.curr_position.y - goal_point[1]) **2 + (self.curr_position.z - goal_point[2])**2)**0.5
        return error
    def set_position(self, position):

        if self.state.mode != "OFFBOARD":
            rospy.loginfo("switch offboard")
            offboard_req = SetModeRequest()
            offboard_req.custom_mode = "OFFBOARD"
            self.set_mode_client(offboard_req)
        if self.if_armed == False:
            response = self.arming_client.call(True)
            if response.success:
                rospy.loginfo('success')
                self.if_armed = True
            else:
                rospy.loginfo(response)        
        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.frame_id = "map"
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = position[0]
        position_msg.pose.position.y = position[1]
        position_msg.pose.position.z = position[2]
        position_msg.pose.orientation.w = 1.0
        position_msg.pose.orientation.x = 0.
        position_msg.pose.orientation.y = 0.
        position_msg.pose.orientation.z = 0.
        self.mavros_pub.publish(position_msg)



    # Callback handlers
    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        # FLY TO GOAL POINT
        self.start_pose = self.curr_position #TODO switch for curr vs camera pose
        if self.vicon_pose is not None:
            self.vicon_start_x = self.vicon_pose.translation.x  
            self.vicon_start_y = self.vicon_pose.translation.y 
            self.vicon_start_z = self.vicon_pose.translation.z   
        else:
            self.vicon_start_x = 0
            self.vicon_start_y = 0 
            self.vicon_start_z = 0


        rospy.loginfo('START POSE %s', self.start_pose)
        self.goal_point = [self.start_pose.x - self.vicon_start_x,
                            self.start_pose.y - self.vicon_start_y,  
                            self.start_pose.z - self.vicon_start_z + 1.5]
        self.control_mode = "LAUNCH"
        rospy.loginfo("GOAL %s", self.goal_point)

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        # TEST
        self.control_mode = "TEST"

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        # LAND AT START POSE
        self.control_mode = "LAND"

    def handle_abort(self):
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

    


    def sar_demo(self):
        self.control_mode = "IDLE" # start in this mode
        base_error = 1000

        # SERVICE BASED TRIGGER
        while not rospy.is_shutdown():
            if self.control_mode == "LAUNCH":
                rospy.loginfo("Flying towards launch point")
                self.set_position(self.goal_point)
                self.rate.sleep()
            
            # For Task 3
            elif self.control_mode == "TEST":
                if self.waypoint_curr == len(self.waypoint_queue):
                    self.control_mode == "LAND"

                # curr_WP is 1x3 np.array
                else:
                    curr_WP = self.waypoint_queue[self.waypoint_curr,:]
                    # rospy.loginfo(curr_WP)

                    self.set_position(curr_WP) 
                    wp_error = self.tolerance_error(curr_WP) # sending in curr goal point to compare against position
                    # rospy.loginfo(wp_error)
                    self.rate.sleep()
                    if wp_error < 0.05:
                        rospy.loginfo("going to next waypoint")
                        self.waypoint_curr += 1
                        if self.waypoint_curr == len(self.waypoint_queue):
                            self.control_mode == "LAND"
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
                self.set_position([self.curr_position.x, self.curr_position.y, self.start_pose.z])
                base_error = self.tolerance_error([self.curr_position.x, self.curr_position.y, self.start_pose.z])
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
        controller.sar_demo()
        

    except rospy.ROSInterruptException:
            pass