;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::AprilTagPosition)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'AprilTagPosition (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::APRILTAGPOSITION")
  (make-package "CUSTOM_MSG::APRILTAGPOSITION"))

(in-package "ROS")
;;//! \htmlinclude AprilTagPosition.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass custom_msg::AprilTagPosition
  :super ros::object
  :slots (_header _id _camera_position _world_position ))

(defmethod custom_msg::AprilTagPosition
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:camera_position __camera_position) (instance geometry_msgs::Point :init))
    ((:world_position __world_position) (instance geometry_msgs::Point :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _camera_position __camera_position)
   (setq _world_position __world_position)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:camera_position
   (&rest __camera_position)
   (if (keywordp (car __camera_position))
       (send* _camera_position __camera_position)
     (progn
       (if __camera_position (setq _camera_position (car __camera_position)))
       _camera_position)))
  (:world_position
   (&rest __world_position)
   (if (keywordp (car __world_position))
       (send* _world_position __world_position)
     (progn
       (if __world_position (setq _world_position (car __world_position)))
       _world_position)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _id
    4
    ;; geometry_msgs/Point _camera_position
    (send _camera_position :serialization-length)
    ;; geometry_msgs/Point _world_position
    (send _world_position :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _id
       (write-long _id s)
     ;; geometry_msgs/Point _camera_position
       (send _camera_position :serialize s)
     ;; geometry_msgs/Point _world_position
       (send _world_position :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/Point _camera_position
     (send _camera_position :deserialize buf ptr-) (incf ptr- (send _camera_position :serialization-length))
   ;; geometry_msgs/Point _world_position
     (send _world_position :deserialize buf ptr-) (incf ptr- (send _world_position :serialization-length))
   ;;
   self)
  )

(setf (get custom_msg::AprilTagPosition :md5sum-) "bd61561e1ae547a5df703150da4f40e2")
(setf (get custom_msg::AprilTagPosition :datatype-) "custom_msg/AprilTagPosition")
(setf (get custom_msg::AprilTagPosition :definition-)
      "# AprilTagPosition.msg

# ROS message for representing the position of an AprilTag
# relative to the camera and in the world coordinate frame

std_msgs/Header header              # Standard ROS message header

int32 id                  # ID of the AprilTag

geometry_msgs/Point camera_position  # Position of the AprilTag relative to the camera
geometry_msgs/Point world_position   # Position of the AprilTag in the world coordinate frame

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :custom_msg/AprilTagPosition "bd61561e1ae547a5df703150da4f40e2")


