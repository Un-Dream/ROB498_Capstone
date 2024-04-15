#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseArray

grid_size = 7
step_size = 0.5
pattern = 'lawnmower'
height = 0.4

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
            boundary = 4.5

            xyz_list = []
            for row in range(num_rows+1):  # Include the last row
                if row % 2 == 0:
                    for col in range(num_cols+1):  # Include the last column
                        x = (col - num_cols/2) * col_step * (boundary / (num_cols * col_step))
                        y = (row - num_rows/2) * row_step * (boundary / (num_rows * row_step))
                        z = height
                        xyz_list.append([x, y, z])
                else:
                    for col in range(num_cols, -1, -1):  # Include the last column
                        x = (col - num_cols/2) * col_step * (boundary / (num_cols * col_step))
                        y = (row - num_rows/2) * row_step * (boundary / (num_rows * row_step))
                        z = height
                        xyz_list.append([x, y, z])
                    z = height
                    xyz_list.append([x, y, z])
            
            # Convert xyz_list to PoseArray message
            for xyz in xyz_list:
                pose = Pose()
                pose.position.x = xyz[0]
                pose.position.y = xyz[1]
                pose.position.z = xyz[2]
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
