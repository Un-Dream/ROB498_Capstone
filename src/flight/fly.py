#!/usr/bin/env python3

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



class Controller:
    def __init__(self):
        rospy.init_node('fly', anonymous = True)
        # self.rate = rospy.Rate(60) #10 Hz # TODO assign a callback, in that callback publish waypoint constantly at that rate, check mode at the same time
        # don't pub odom in that topic
        # basically move set point out of our loop into the rate loop
        

        # creating publishers and subcribers

        # mavros / imu data
        self.pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10)
        self.sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.position_callback)
        self.odompub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size = 10)
        
        # vicon
        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', PoseStamped, self.position_callback)

        # camera
        rospy.loginfo('camera')
        self.camera = rospy.Subscriber('/camera/odom/sample', Odometry, self.camera_callback)
        

        # Internal variables
        self.camera_pose = None
        self.camera_data_full = None
        self.curr_position = None
        
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
        

        self.control_mode = None

        # # TODO add transforms
        # # self.add_transform() # run before flight

        rospy.loginfo("finish startup")

    

            






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
        self.odompub.publish(odom_msg)

    def add_transform(self):
        # run this before flight
        # add transform of original pose 

        # self. current pose and camera coords
        trans = (0,2,0) #None # (0, 0, 0), #camera coords - the imu
        # IN QUATERNION BTW
        rot = (0,0,0,1) #None # (orientation.x, orientation.y, orientation.z, orientation.w)

        child_frame_id = "camera"
        parent_frame_id = "imu"

        br = tf.TransformBroadcaster()
        br.sendTransform(trans,
                     rot,
                     rospy.Time.now(),
                     child_frame_id,
                    parent_frame_id)

        rospy.loginfo("added")

    def get_transform(self):
        # translation and rotation
        child_frame_id = "/camera"
        parent_frame_id = "/imu"
        listener = tf.TransformListener()
        # try:
        (trans, rot) = listener.lookupTransform(parent_frame_id, child_frame_id, rospy.Time(0))
        print("Translation:", trans)
        print("Rotation:", rot)
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     continue

    def position_callback(self, msg):
        # update current position
        self.curr_position = msg.pose.position

    def state_callback(self, state):
        self.state = state

    def fly_to_base(self):
        # send to local position so PoseStamped data type
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

        # self.rate.sleep()
        
        position_msg = PoseStamped()
        position_msg.header = Header()
        position_msg.header.stamp = rospy.Time.now()
        position_msg.pose.position.x = position[0]
        position_msg.pose.position.y = position[1]
        position_msg.pose.position.z = position[2]
        self.pub.publish(position_msg)

    def tolerance_error(self, goal_point):
        # error = ((self.camera_pose.x - goal_point[0]) ** 2 + (self.camera_pose.y - goal_point[1]) **2 + (self.camera_pose.z - goal_point[2])**2)**0.5
        
        error = ((self.camera_pose.z - goal_point[2])**2)**0.5
        
        return error




    def run(self):
        rate = rospy.Rate(60)
        # rospy.spin()
        goal_point = self.goal_point
        error = 1000
        # base_error = 1000
        while not rospy.is_shutdown():

            if self.control_mode == "FLY":
                rospy.loginfo("Flying towards point")

                # rospy.loginfo("Flying towards point")
                # rospy.loginfo("Current pose: %s", controller.camera_pose)
                controller.set_position(goal_point)
                error = controller.tolerance_error(goal_point)
                start_time = time.time()
                rate.sleep()

                if error < 0.05:
                    self.control_mode = "HOVER"
            
            # init_t = rospy.Time.now()
            if self.control_mode == "HOVER":
                rospy.loginfo("hover")
                # rospy.loginfo("Current pose: %s", controller.curr_position)
                controller.set_position(goal_point)
                # rospy.loginfo("Current pose: %s", controller.curr_position)
                rate.sleep()

                if time.time() - start_time > 10:
                    self.control_mode = "RETURN"



            if self.control_mode == "RETURN":
                rospy.loginfo("fly home")

                controller.fly_to_base()
                base_error = controller.tolerance_error([controller.start_pose.x, controller.start_pose.y, controller.start_pose.z])
                rospy.loginfo("Current pose: %s", controller.camera_pose)

                rate.sleep()
                if base_error < 0.05:
                    self.control_mode = "END"


            if self.control_mode == "END":
                rospy.loginfo("end")

                rospy.spin()


            rate.sleep()


            
      


if __name__  == "__main__":
    try: 

        # controller = Controller()
        # while True:
        #     controller.add_transform()

        #     pass
        # controller.get_transform()
        controller = Controller()
        controller.goal_point = [0, 0, controller.start_pose.z+0.4]
        controller.control_mode = "FLY"
        controller.run()

            



            

    except rospy.ROSInterruptException:
        pass