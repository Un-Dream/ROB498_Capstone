#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseArray

grid_size = 5
step_size = 1
pattern = 'lawnmower'
height = 1

def waypoints():
    # Initialize the ROS node
    rospy.init_node('waypoints', anonymous=True)

    # Create a publisher for the PoseArray topic
    pose_array_pub = rospy.Publisher('/comm/waypoints', PoseArray, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        pose_array_msg = PoseArray()

        if pattern == "lawnmower":
            # Create a PoseArray message
            # Generate lawnmower pattern
            num_rows = grid_size
            num_cols = grid_size
            row_step = step_size
            col_step = step_size

            for row in range(num_rows):
                for col in range(num_cols):
                    pose = Pose()
                    pose.position.x = col * col_step
                    pose.position.y = row * row_step if row % 2 == 0 else (num_cols - col - 1) * col_step
                    pose.position.z = height
                    pose.orientation.w = 1.0  # Identity quaternion
                    pose_array_msg.poses.append(pose)

        # Publish the PoseArray message
        pose_array_pub.publish(pose_array_msg)
        rospy.loginfo(pose_array_msg)
        # rospy.loginfo(pose_array_msg)
        # Sleep according to the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        waypoints()
    except rospy.ROSInterruptException:
        pass

  
  # estimate of our own pose vs GT
  # plot trajectory 
  # all the points coming in
  # plot vicon, estimate and waypoints
