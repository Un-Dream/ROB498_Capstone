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
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection

class Eval:
    def __init__(self):
        rospy.init_node('eval', anonymous=True)
        node_name = 'rob498_drone_05'



        self.vicon_pose = None
        self.waypoints = None
        self.waypoint_saved = False

        self.timer = None
        
        self.positions_vicon = []
        self.positions_waypoints = []
        self.positions_est = []

        self.tagGT = np.array([[2.4384,0,0],[2.4384+0.12375,-0.0217,0],[0, -2.4384,0],[0.12375,-2.4384-0.0217, 0],[-2.4384, 2.4384, 0],[-2.4384+0.12375, 2.4384-0.0217, 0]])

        self.srv_test = rospy.Service(node_name + '/comm/test', Empty, self.callback_test)


        self.tag2_world_position = []
        self.tag3_world_position = []
        self.tag4_world_position = []
        self.tag5_world_position = []
        self.tag6_world_position = []
        self.tag7_world_position = []
        self.tag2_euclid_error = []
        self.tag3_euclid_error = []
        self.tag4_euclid_error = []
        self.tag5_euclid_error = []
        self.tag6_euclid_error = []
        self.tag7_euclid_error = []
        self.tag2_axis_error = []
        self.tag3_axis_error = []
        self.tag4_axis_error = []
        self.tag5_axis_error = []
        self.tag6_axis_error = []
        self.tag7_axis_error = []

        self.vicon = rospy.Subscriber('/vicon/ROB498_Drone/ROB498_Drone', TransformStamped, self.vicon_callback)
        self.waypoints_sub = rospy.Subscriber('/comm/waypoints', PoseArray, self.callback_waypoints)
        self.mavros_pose = rospy.Subscriber('/mavros/odometry/in', Odometry, self.mavros_callback)
        # self.april_tag = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        self.tagPose = rospy.Subscriber('/tag_position/world', AprilTagDetection, self.tagPose_callback)


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

    def tagPose_callback(self, msg):
        try:
            tagId = msg.id[0]
            pose = msg.pose.pose.pose.position
            x = pose.x
            y = pose.y
            z = pose.z
            est = np.array([x,y,z])
            gt = self.tagGT[tagId-2]
            euclid_error = np.sqrt(np.sum(np.square(est - gt)))
            axis_error_x = abs(est[0] - gt[0])
            axis_error_y = abs(est[1] - gt[1])
            axis_error_z = abs(est[2] - gt[2])
            # rospy.loginfo(f'For the tag id {tagId}, the error is {euclid_error} meters')

            # Switch case to check tagId and add position to corresponding list
            if tagId == 2:
                self.tag2_world_position.append([x, y, z])
                self.tag2_euclid_error.append(euclid_error)
                self.tag2_axis_error.append([axis_error_x, axis_error_y, axis_error_z])
            elif tagId == 3:
                self.tag3_world_position.append([x, y, z])
                self.tag3_euclid_error.append(euclid_error)
                self.tag3_axis_error.append([axis_error_x, axis_error_y, axis_error_z])
            elif tagId == 4:
                self.tag4_world_position.append([x, y, z])
                self.tag4_euclid_error.append(euclid_error)
                self.tag4_axis_error.append([axis_error_x, axis_error_y, axis_error_z])
            elif tagId == 5:
                self.tag5_world_position.append([x, y, z])
                self.tag5_euclid_error.append(euclid_error)
                self.tag5_axis_error.append([axis_error_x, axis_error_y, axis_error_z])
            elif tagId == 6:
                self.tag6_world_position.append([x, y, z])
                self.tag6_euclid_error.append(euclid_error)
                self.tag6_axis_error.append([axis_error_x, axis_error_y, axis_error_z])
            elif tagId == 7:
                self.tag7_world_position.append([x, y, z])
                self.tag7_euclid_error.append(euclid_error)
                self.tag7_axis_error.append([axis_error_x, axis_error_y, axis_error_z])
            else:
                rospy.loginfo(f'Invalid tag id: {tagId}')

        except: 
            rospy.loginfo('tag position callback error - no data')


        


    def plot(self, vicon=False, waypoints=True, estimate=True):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)

        # Add major lines every 1
        ax.set_xticks(np.arange(-4, 4.5, 1))
        ax.set_yticks(np.arange(-4, 4.5, 1))

        # Add minor lines every 0.5
        ax.set_xticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax.set_yticks(np.arange(-4, 4.5, 0.5), minor=True)

        ax.grid(True, which='both')

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


        # Plot top-down view of x and y
        fig_top_xy = plt.figure()
        ax_top_xy = fig_top_xy.add_subplot(111)
        ax_top_xy.set_xlabel('X')
        ax_top_xy.set_ylabel('Y')
        ax_top_xy.set_xlim(-4, 4)
        ax_top_xy.set_ylim(-4, 4)
        ax_top_xy.set_xticks(np.arange(-4, 4.5, 1))
        ax_top_xy.set_yticks(np.arange(-4, 4.5, 1))
        ax_top_xy.set_xticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax_top_xy.set_yticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax_top_xy.grid(True, which='both')
        # Plot ground truth positions
        if len(self.positions_vicon) > 0:
            ax_top_xy.scatter(*zip(*[[pos[0], pos[1]] for pos in self.positions_vicon]), marker='.', s=2)
        if len(self.positions_waypoints) > 0:
            ax_top_xy.scatter(*zip(*[[pos[0], pos[1]] for pos in self.positions_waypoints]), c='b', marker='.', s=2, label='Waypoints')
        if len(self.positions_est) > 0:
            ax_top_xy.scatter(*zip(*[[pos[0], pos[1]] for pos in self.positions_est]), c='g', marker='.', s=2, label='Estimated')
        ax_top_xy.legend()
        plt.savefig('pose_over_time_top_down.png')

    def plot_april_tags(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)
        ax.set_zlim(-4, 4)
        ax.set_xticks(np.arange(-4, 4.5, 1))
        ax.set_yticks(np.arange(-4, 4.5, 1))
        ax.set_zticks(np.arange(-4, 4.5, 1))
        ax.set_xticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax.set_yticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax.set_zticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax.grid(True, which='both')

        # Plot ground truth positions
        ax.scatter(*zip(*self.tagGT), c='r', marker='o', label='Ground Truth')

        # Plot tag positions
        if len(self.tag2_world_position) > 0:
            ax.scatter(*zip(*self.tag2_world_position), c='b', marker='o', label='Tag 2', s=1)
        if len(self.tag3_world_position) > 0:
            ax.scatter(*zip(*self.tag3_world_position), c='g', marker='o', label='Tag 3', s=1)
        if len(self.tag4_world_position) > 0:
            ax.scatter(*zip(*self.tag4_world_position), c='m', marker='o', label='Tag 4', s=1)
        if len(self.tag5_world_position) > 0:
            ax.scatter(*zip(*self.tag5_world_position), c='c', marker='o', label='Tag 5', s=1)
        if len(self.tag6_world_position) > 0:
            ax.scatter(*zip(*self.tag6_world_position), c='y', marker='o', label='Tag 6', s=1)
        if len(self.tag7_world_position) > 0:
            ax.scatter(*zip(*self.tag7_world_position), c='k', marker='o', label='Tag 7', s=1)

        ax.legend()
        plt.savefig('april_tags.png')

        # Plot top-down view
        fig_top = plt.figure()
        ax_top = fig_top.add_subplot(111)
        ax_top.set_xlabel('X')
        ax_top.set_ylabel('Y')
        ax_top.set_xlim(-4, 4)
        ax_top.set_ylim(-4, 4)
        ax_top.set_xticks(np.arange(-4, 4.5, 1))
        ax_top.set_yticks(np.arange(-4, 4.5, 1))
        ax_top.set_xticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax_top.set_yticks(np.arange(-4, 4.5, 0.5), minor=True)
        ax_top.grid(True, which='both')

        # Plot ground truth positions
        ax_top.scatter(*zip(*self.tagGT[:, :2]), c='r', marker='o', label='Ground Truth')
        
        # Plot tag positions
        if len(self.tag2_world_position) > 0:
            ax_top.scatter(*zip(*[[pos[0], pos[1]] for pos in self.tag2_world_position]), c='b', marker='o', label='Tag 2', s=1)
        if len(self.tag3_world_position) > 0:
            ax_top.scatter(*zip(*[[pos[0], pos[1]] for pos in self.tag3_world_position]), c='g', marker='o', label='Tag 3', s=1)
        if len(self.tag4_world_position) > 0:
            ax_top.scatter(*zip(*[[pos[0], pos[1]] for pos in self.tag4_world_position]), c='m', marker='o', label='Tag 4', s=1)
        if len(self.tag5_world_position) > 0:
            ax_top.scatter(*zip(*[[pos[0], pos[1]] for pos in self.tag5_world_position]), c='c', marker='o', label='Tag 5', s=1)
        if len(self.tag6_world_position) > 0:
            ax_top.scatter(*zip(*[[pos[0], pos[1]] for pos in self.tag6_world_position]), c='y', marker='o', label='Tag 6', s=1)
        if len(self.tag7_world_position) > 0:
            ax_top.scatter(*zip(*[[pos[0], pos[1]] for pos in self.tag7_world_position]), c='k', marker='o', label='Tag 7', s=1)

        ax_top.legend()
        plt.savefig('april_tags_top.png')

        # Plot euclidean errors
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_xlabel('Time')
        ax.set_ylabel('Euclidean Error (m)')
        ax.grid(True)  # Add grid
        if len(self.tag2_euclid_error) > 0:
            ax.plot(range(len(self.tag2_euclid_error)), self.tag2_euclid_error, label='Tag 2')
        if len(self.tag3_euclid_error) > 0:
            ax.plot(range(len(self.tag3_euclid_error)), self.tag3_euclid_error, label='Tag 3')
        if len(self.tag4_euclid_error) > 0:
            ax.plot(range(len(self.tag4_euclid_error)), self.tag4_euclid_error, label='Tag 4')
        if len(self.tag5_euclid_error) > 0:
            ax.plot(range(len(self.tag5_euclid_error)), self.tag5_euclid_error, label='Tag 5')
        if len(self.tag6_euclid_error) > 0:
            ax.plot(range(len(self.tag6_euclid_error)), self.tag6_euclid_error, label='Tag 6')
        if len(self.tag7_euclid_error) > 0:
            ax.plot(range(len(self.tag7_euclid_error)), self.tag7_euclid_error, label='Tag 7')
        ax.legend()
        plt.savefig('euclidean_errors.png')

        # Plot axis errors
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
        fig.suptitle('Axis Errors')
        ax1.set_ylabel('X Error (m)')
        ax2.set_ylabel('Y Error (m)')
        ax3.set_ylabel('Z Error (m)')
        ax3.set_xlabel('Time')
        ax1.grid(True)  # Add grid
        ax2.grid(True)  # Add grid
        ax3.grid(True)  # Add grid
        if len(self.tag2_axis_error) > 0:
            tag2_axis_error = np.array(self.tag2_axis_error)
            ax1.plot(range(len(tag2_axis_error)), tag2_axis_error[:, 0], label='Tag 2')
            ax2.plot(range(len(tag2_axis_error)), tag2_axis_error[:, 1], label='Tag 2')
            ax3.plot(range(len(tag2_axis_error)), tag2_axis_error[:, 2], label='Tag 2')
        if len(self.tag3_axis_error) > 0:
            tag3_axis_error = np.array(self.tag3_axis_error)
            ax1.plot(range(len(tag3_axis_error)), tag3_axis_error[:, 0], label='Tag 3')
            ax2.plot(range(len(tag3_axis_error)), tag3_axis_error[:, 1], label='Tag 3')
            ax3.plot(range(len(tag3_axis_error)), tag3_axis_error[:, 2], label='Tag 3')
        if len(self.tag4_axis_error) > 0:
            tag4_axis_error = np.array(self.tag4_axis_error)
            ax1.plot(range(len(tag4_axis_error)), tag4_axis_error[:, 0], label='Tag 4')
            ax2.plot(range(len(tag4_axis_error)), tag4_axis_error[:, 1], label='Tag 4')
            ax3.plot(range(len(tag4_axis_error)), tag4_axis_error[:, 2], label='Tag 4')
        if len(self.tag5_axis_error) > 0:
            tag5_axis_error = np.array(self.tag5_axis_error)
            ax1.plot(range(len(tag5_axis_error)), tag5_axis_error[:, 0], label='Tag 5')
            ax2.plot(range(len(tag5_axis_error)), tag5_axis_error[:, 1], label='Tag 5')
            ax3.plot(range(len(tag5_axis_error)), tag5_axis_error[:, 2], label='Tag 5')
        if len(self.tag6_axis_error) > 0:
            tag6_axis_error = np.array(self.tag6_axis_error)
            ax1.plot(range(len(tag6_axis_error)), tag6_axis_error[:, 0], label='Tag 6')
            ax2.plot(range(len(tag6_axis_error)), tag6_axis_error[:, 1], label='Tag 6')
            ax3.plot(range(len(tag6_axis_error)), tag6_axis_error[:, 2], label='Tag 6')
        if len(self.tag7_axis_error) > 0:
            tag7_axis_error = np.array(self.tag7_axis_error)
            ax1.plot(range(len(tag7_axis_error)), tag7_axis_error[:, 0], label='Tag 7')
            ax2.plot(range(len(tag7_axis_error)), tag7_axis_error[:, 1], label='Tag 7')
            ax3.plot(range(len(tag7_axis_error)), tag7_axis_error[:, 2], label='Tag 7')
        ax1.legend()
        ax2.legend()
        ax3.legend()
        plt.savefig('axis_errors.png')


if __name__  == "__main__":
    try: 
        test = Eval()

        while not rospy.is_shutdown():
            test.plot()
            test.plot_april_tags()
        rospy.spin()
    except rospy.ROSInterruptException:
            pass
    
    
            
