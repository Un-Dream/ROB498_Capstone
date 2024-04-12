#!/usr/bin/env python3

import math
import tf
import rospy
import mavros_msgs
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt


from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool, CommandBoolRequest, SetModeRequest
from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped, TransformStamped, Pose, PoseArray
from std_msgs.msg import Header
from std_srvs.srv import Empty, EmptyResponse

from mpl_toolkits.mplot3d import Axes3D  # Importing 3D axes


class Eval:
    def __init__(self):
        rospy.init_node('eval', anonymous=True)
        node_name = 'rob498_drone_05'
        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.vicon_callback)
        self.waypoints_sub = rospy.Subscriber('/comm/waypoints', PoseArray, self.callback_waypoints)
        self.mavros_pose = rospy.Subscriber('/mavros/odometry/in', Odometry, self.mavros_callback)


        self.vicon_pose = None
        self.waypoints = None
        self.waypoint_saved = False

        self.timer = None
        
        self.positions_vicon = []
        self.positions_waypoints = []
        self.positions_est = []

        
        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)

    def callback_test(self, request):
        self.handle_test()
        # self.timer = rospy.Timer(rospy.Duration(2), self.plot())
        return EmptyResponse()
    
    def handle_test(self):
        self.positions_vicon = []
        self.positions_waypoints = []
        self.positions_est = []
        rospy.loginfo("start to log position")

    def mavros_callback(self,odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        z = odom_msg.pose.pose.position.z
        self.positions_est.append((x, y, z))


    def vicon_callback(self, msg):
        # update current position
        try:
            self.vicon_pose = msg.transform
            position = msg.transform.translation
            self.positions_vicon.append((position.x, position.y, position.z))
            
        except: 
            rospy.loginfo('vicon callback error - no data')


    def callback_waypoints(self, pose_array_msg):

        if self.waypoint_saved == False:
            # Iterate through each pose in the PoseArray message
            for pose in pose_array_msg.poses:
                # Extract x, y, z components of the position
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z
                # Append [x, y, z] to the positions list
                self.positions_waypoints.append([x, y, z])
                self.waypoint_saved = True


    def plot(self, vicon=False, waypoints=True, estimate=True):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        if vicon:
            ax.plot(*zip(* self.positions_vicon), label = "vicon")
        rospy.loginfo(self.positions_waypoints)
        rospy.loginfo(len(self.positions_waypoints))
        if waypoints and len(self.positions_waypoints)>0:
            ax.plot(*zip(* self.positions_waypoints), label = "waypoints")

        if estimate and len(self.positions_est)>0:
            ax.plot(*zip(* self.positions_est), label = "estimated")

        ax.legend()
        plt.savefig('pose_over_time.png')


if __name__  == "__main__":
    try: 
        test = Eval()

        while not rospy.is_shutdown():
            test.plot()
        rospy.spin()
    except rospy.ROSInterruptException:
            pass
    
    
            
