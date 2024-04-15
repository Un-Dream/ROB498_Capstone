; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude AprilTagPosition.msg.html

(cl:defclass <AprilTagPosition> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (camera_position
    :reader camera_position
    :initarg :camera_position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (world_position
    :reader world_position
    :initarg :world_position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass AprilTagPosition (<AprilTagPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AprilTagPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AprilTagPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<AprilTagPosition> is deprecated: use custom_msg-msg:AprilTagPosition instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AprilTagPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:header-val is deprecated.  Use custom_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <AprilTagPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:id-val is deprecated.  Use custom_msg-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'camera_position-val :lambda-list '(m))
(cl:defmethod camera_position-val ((m <AprilTagPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:camera_position-val is deprecated.  Use custom_msg-msg:camera_position instead.")
  (camera_position m))

(cl:ensure-generic-function 'world_position-val :lambda-list '(m))
(cl:defmethod world_position-val ((m <AprilTagPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:world_position-val is deprecated.  Use custom_msg-msg:world_position instead.")
  (world_position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AprilTagPosition>) ostream)
  "Serializes a message object of type '<AprilTagPosition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'world_position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AprilTagPosition>) istream)
  "Deserializes a message object of type '<AprilTagPosition>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'world_position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AprilTagPosition>)))
  "Returns string type for a message object of type '<AprilTagPosition>"
  "custom_msg/AprilTagPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AprilTagPosition)))
  "Returns string type for a message object of type 'AprilTagPosition"
  "custom_msg/AprilTagPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AprilTagPosition>)))
  "Returns md5sum for a message object of type '<AprilTagPosition>"
  "bd61561e1ae547a5df703150da4f40e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AprilTagPosition)))
  "Returns md5sum for a message object of type 'AprilTagPosition"
  "bd61561e1ae547a5df703150da4f40e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AprilTagPosition>)))
  "Returns full string definition for message of type '<AprilTagPosition>"
  (cl:format cl:nil "# AprilTagPosition.msg~%~%# ROS message for representing the position of an AprilTag~%# relative to the camera and in the world coordinate frame~%~%std_msgs/Header header              # Standard ROS message header~%~%int32 id                  # ID of the AprilTag~%~%geometry_msgs/Point camera_position  # Position of the AprilTag relative to the camera~%geometry_msgs/Point world_position   # Position of the AprilTag in the world coordinate frame~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AprilTagPosition)))
  "Returns full string definition for message of type 'AprilTagPosition"
  (cl:format cl:nil "# AprilTagPosition.msg~%~%# ROS message for representing the position of an AprilTag~%# relative to the camera and in the world coordinate frame~%~%std_msgs/Header header              # Standard ROS message header~%~%int32 id                  # ID of the AprilTag~%~%geometry_msgs/Point camera_position  # Position of the AprilTag relative to the camera~%geometry_msgs/Point world_position   # Position of the AprilTag in the world coordinate frame~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AprilTagPosition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'world_position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AprilTagPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'AprilTagPosition
    (cl:cons ':header (header msg))
    (cl:cons ':id (id msg))
    (cl:cons ':camera_position (camera_position msg))
    (cl:cons ':world_position (world_position msg))
))
