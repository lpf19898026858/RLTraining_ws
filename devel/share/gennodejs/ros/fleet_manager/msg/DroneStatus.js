// Auto-generated. Do not edit!

// (in-package fleet_manager.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class DroneStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.status = null;
      this.last_completed_task_index = null;
      this.last_known_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = '';
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('last_completed_task_index')) {
        this.last_completed_task_index = initObj.last_completed_task_index
      }
      else {
        this.last_completed_task_index = 0;
      }
      if (initObj.hasOwnProperty('last_known_pose')) {
        this.last_known_pose = initObj.last_known_pose
      }
      else {
        this.last_known_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DroneStatus
    // Serialize message field [drone_id]
    bufferOffset = _serializer.string(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [last_completed_task_index]
    bufferOffset = _serializer.int32(obj.last_completed_task_index, buffer, bufferOffset);
    // Serialize message field [last_known_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.last_known_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DroneStatus
    let len;
    let data = new DroneStatus(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [last_completed_task_index]
    data.last_completed_task_index = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [last_known_pose]
    data.last_known_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.drone_id);
    return length + 65;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fleet_manager/DroneStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4399a33682bd42bcbd894637e74a998e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # [修改] 在消息定义中添加常量
    # Drone Status Constants
    uint8 UNKNOWN=0
    uint8 ACTIVE=1
    uint8 NEEDS_BACKUP=2
    uint8 OFFLINE=3
    
    # Message Fields
    string drone_id
    uint8 status
    int32 last_completed_task_index
    geometry_msgs/Pose last_known_pose
    
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
    const resolved = new DroneStatus(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = ''
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.last_completed_task_index !== undefined) {
      resolved.last_completed_task_index = msg.last_completed_task_index;
    }
    else {
      resolved.last_completed_task_index = 0
    }

    if (msg.last_known_pose !== undefined) {
      resolved.last_known_pose = geometry_msgs.msg.Pose.Resolve(msg.last_known_pose)
    }
    else {
      resolved.last_known_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

// Constants for message
DroneStatus.Constants = {
  UNKNOWN: 0,
  ACTIVE: 1,
  NEEDS_BACKUP: 2,
  OFFLINE: 3,
}

module.exports = DroneStatus;
