// Auto-generated. Do not edit!

// (in-package custom_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ResetEpisodeRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetEpisodeRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetEpisodeRequest
    let len;
    let data = new ResetEpisodeRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'custom_msgs/ResetEpisodeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # custom_msgs/srv/ResetEpisode.srv
    
    # --- Request ---
    # 请求可以是空的，因为我们只是想触发一个重置
    # 可以加一个bool来支持课程学习，但现在先保持简单
    # bool use_curriculum
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetEpisodeRequest(null);
    return resolved;
    }
};

class ResetEpisodeResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.start_pose = null;
      this.goal_position = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
      if (initObj.hasOwnProperty('start_pose')) {
        this.start_pose = initObj.start_pose
      }
      else {
        this.start_pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('goal_position')) {
        this.goal_position = initObj.goal_position
      }
      else {
        this.goal_position = new geometry_msgs.msg.Point();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ResetEpisodeResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [start_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.start_pose, buffer, bufferOffset);
    // Serialize message field [goal_position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.goal_position, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ResetEpisodeResponse
    let len;
    let data = new ResetEpisodeResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [start_pose]
    data.start_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_position]
    data.goal_position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 85;
  }

  static datatype() {
    // Returns string type for a service object
    return 'custom_msgs/ResetEpisodeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e566690f9c9e0029132e05fa58e0f836';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    # --- Response ---
    # Unity找到的安全点将在这里返回
    bool success          # 标记重置是否成功
    string message        # 附带信息 (例如，失败原因)
    geometry_msgs/Pose start_pose
    geometry_msgs/Point goal_position
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ResetEpisodeResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    if (msg.start_pose !== undefined) {
      resolved.start_pose = geometry_msgs.msg.Pose.Resolve(msg.start_pose)
    }
    else {
      resolved.start_pose = new geometry_msgs.msg.Pose()
    }

    if (msg.goal_position !== undefined) {
      resolved.goal_position = geometry_msgs.msg.Point.Resolve(msg.goal_position)
    }
    else {
      resolved.goal_position = new geometry_msgs.msg.Point()
    }

    return resolved;
    }
};

module.exports = {
  Request: ResetEpisodeRequest,
  Response: ResetEpisodeResponse,
  md5sum() { return 'e566690f9c9e0029132e05fa58e0f836'; },
  datatype() { return 'custom_msgs/ResetEpisode'; }
};
