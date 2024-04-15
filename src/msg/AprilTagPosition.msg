# AprilTagPosition.msg

# ROS message for representing the position of an AprilTag
# relative to the camera and in the world coordinate frame

std_msgs/Header header              # Standard ROS message header

int32 id                  # ID of the AprilTag

geometry_msgs/Point camera_position  # Position of the AprilTag relative to the camera
geometry_msgs/Point world_position   # Position of the AprilTag in the world coordinate frame
