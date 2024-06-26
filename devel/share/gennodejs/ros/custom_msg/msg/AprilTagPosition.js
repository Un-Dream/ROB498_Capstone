// Auto-generated. Do not edit!

// (in-package custom_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class AprilTagPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.id = null;
      this.camera_position = null;
      this.world_position = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('camera_position')) {
        this.camera_position = initObj.camera_position
      }
      else {
        this.camera_position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('world_position')) {
        this.world_position = initObj.world_position
      }
      else {
        this.world_position = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AprilTagPosition
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [camera_position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.camera_position, buffer, bufferOffset);
    // Serialize message field [world_position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.world_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AprilTagPosition
    let len;
    let data = new AprilTagPosition(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [camera_position]
    data.camera_position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [world_position]
    data.world_position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 52;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg/AprilTagPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bd61561e1ae547a5df703150da4f40e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # AprilTagPosition.msg
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AprilTagPosition(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.camera_position !== undefined) {
      resolved.camera_position = geometry_msgs.msg.Point.Resolve(msg.camera_position)
    }
    else {
      resolved.camera_position = new geometry_msgs.msg.Point()
    }

    if (msg.world_position !== undefined) {
      resolved.world_position = geometry_msgs.msg.Point.Resolve(msg.world_position)
    }
    else {
      resolved.world_position = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = AprilTagPosition;
