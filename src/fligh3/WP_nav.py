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
        
        # rospy.init_node('hover', anonymous = True)
        node_name = 'rob498_drone_05'
        rospy.init_node(node_name) 
        self.rate = rospy.Rate(60)
        # mavros publishers
        self.mavros_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.mavros_odom = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size = 10)

        ### Subscriber Banks
        # camera data
        self.camera_odom = rospy.Subscriber('/camera/odom/sample', Odometry, self.camera_callback)        
        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.vicon_callback)
        
        # Internal variables
        self.vicon_pose = None
        self.camera_pose = None
        self.camera_data_full = None
        self.curr_position = None
        self.control_mode = "IDLE"
        self.waypoints = None
        self.WAYPOINTS_RECEIVED = False
        self.waypoint_curr = 0

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
        self.start_pose  = None
        self.goal_point = None
        self.vicon_start_z = 0

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
        try:
            self.vicon_pose = msg.transform
        except: 
            rospy.loginfo('vicon callback error - no data')

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
        position_msg.pose.pose.position.x = position[0]
        position_msg.pose.pose.position.y = position[1]
        position_msg.pose.pose.position.z = position[2]
        self.mavros_pub.publish(position_msg)

    # Callback handlers
    def handle_launch(self):
        print('Launch Requested. Your drone should take off.')
        # FLY TO GOAL POINT
        self.control_mode = "FLY"
        # self.start_pose = self.camera_pose
        if self.vicon_pose is not None:
            self.vicon_start_z = self.vicon_pose.translation.z   
        else:
            self.vicon_start_z = 0

        rospy.loginfo('START POSE %s', self.start_pose)
        self.goal_point = [self.start_pose.x, self.start_pose.y, self.start_pose.z - self.vicon_start_z + 1.5]

    def handle_test(self):
        print('Test Requested. Your drone should perform the required tasks. Recording starts now.')
        # # HOVER
        self.control_mode = "HOVER"

    def handle_land(self):
        print('Land Requested. Your drone should land.')
        # LAND AT START POSE
        self.control_mode = "LAND"

    def handle_abort(self):
        print('Abort Requested. Your drone should land immediately due to safety considerations')
        # LAND AT CURRENT POSITION
        self.control_mode = "ABORT"

    def handle_task(self):
        print('Task Requested. Should begin search for WP via WP_list.')
        # TASK
        self.control_mode = "TASK"

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
    
    def callback_task(self, request):
        self.handle_task()
        return EmptyResponse()
    
    def jiggle_generator(self, waypoints):
        '''
        Add jiggle points into original waypoints array Nx3
        Output: jiggle_wps:
                    np.array in 5Nx3
                    [* * * * *
                    * 4 * 3 *
                    * * 2 * *
                    * 1 * 5 *
                    * * * * *]
                    2 is the original wp
        '''
        jiggle_wps = np.empty((0,3))
        jiggle_dis = 0.1 #meter
        for i in range(np.shape(waypoints)[0]):
            p_2 = waypoints[i]
            jiggle_wps = np.vstack((jiggle_wps, [p_2[0]-jiggle_dis, p_2[1]-jiggle_dis, p_2[2]]))
            jiggle_wps = np.vstack((jiggle_wps, p_2))
            jiggle_wps = np.vstack((jiggle_wps, [p_2[0]+jiggle_dis, p_2[1]+jiggle_dis, p_2[2]]))
            jiggle_wps = np.vstack((jiggle_wps, [p_2[0]-jiggle_dis, p_2[1]+jiggle_dis, p_2[2]]))
            jiggle_wps = np.vstack((jiggle_wps, [p_2[0]+jiggle_dis, p_2[1]-jiggle_dis, p_2[2]]))
        return jiggle_wps

    def callback_waypoints(self, msg):
        '''
        Input: msg in PoseArray
        Output:self.waypoints:
                    np.array in Nx3; storing original WP from msg
        '''
        if self.WAYPOINTS_RECEIVED:
            return
        print('Waypoints Received')
        self.WAYPOINTS_RECEIVED = True
        self.waypoints = np.empty((0,3))
        for pose in msg.poses:
            pos = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.waypoints = np.vstack((self.waypoints, pos))

    def flight_exercise_3(self):
        self.control_mode = "IDLE" # start in this mode
        base_error = 1000

        # waypoint subscribe
        self.waypoints_sub = rospy.Subscriber(self.node_name + '/comm/waypoints', PoseArray, self.callback_waypoints)
        self.waypoints = self.jiggle_generator(self.waypoints)
        # SERVICE BASED TRIGGER
        while not rospy.is_shutdown():
            if self.control_mode == "FLY":
                rospy.loginfo("Flying towards point")
                rospy.loginfo("POSE %s", self.camera_pose.z)

                self.set_position(self.goal_point)
                self.rate.sleep()
            
            # For Task 3
            elif self.control_mode == "TASK":
                rospy.loginfo("task")
                if self.waypoints == None:
                    self.control_mode = "FLY"
                    rospy.loginfo("Flying towards point")
                    rospy.loginfo("POSE %s", self.camera_pose.z)
                    #self.WP = WP1 do we know if advance or when TAs launce is it added to the queue?
                else:
                    # curr_WP is 3x1 np.array
                    curr_WP = self.waypoints[self.waypoint_curr,:]
                    self.waypoint_curr += 1
                    self.set_position(self, curr_WP)
                
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
        controller.flight_exercise_3()
        

    except rospy.ROSInterruptException:
            pass
