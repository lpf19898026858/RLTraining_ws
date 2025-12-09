// Auto-generated. Do not edit!

// (in-package fleet_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let DroneStatus = require('../msg/DroneStatus.js');
let TaskStatus = require('../msg/TaskStatus.js');

//-----------------------------------------------------------

class QueryFleetAndTasksRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QueryFleetAndTasksRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QueryFleetAndTasksRequest
    let len;
    let data = new QueryFleetAndTasksRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'fleet_manager/QueryFleetAndTasksRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 请求部分为空
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QueryFleetAndTasksRequest(null);
    return resolved;
    }
};

class QueryFleetAndTasksResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drones = null;
      this.tasks = null;
    }
    else {
      if (initObj.hasOwnProperty('drones')) {
        this.drones = initObj.drones
      }
      else {
        this.drones = [];
      }
      if (initObj.hasOwnProperty('tasks')) {
        this.tasks = initObj.tasks
      }
      else {
        this.tasks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type QueryFleetAndTasksResponse
    // Serialize message field [drones]
    // Serialize the length for message field [drones]
    bufferOffset = _serializer.uint32(obj.drones.length, buffer, bufferOffset);
    obj.drones.forEach((val) => {
      bufferOffset = DroneStatus.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [tasks]
    // Serialize the length for message field [tasks]
    bufferOffset = _serializer.uint32(obj.tasks.length, buffer, bufferOffset);
    obj.tasks.forEach((val) => {
      bufferOffset = TaskStatus.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type QueryFleetAndTasksResponse
    let len;
    let data = new QueryFleetAndTasksResponse(null);
    // Deserialize message field [drones]
    // Deserialize array length for message field [drones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.drones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.drones[i] = DroneStatus.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [tasks]
    // Deserialize array length for message field [tasks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tasks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tasks[i] = TaskStatus.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.drones.forEach((val) => {
      length += DroneStatus.getMessageSize(val);
    });
    object.tasks.forEach((val) => {
      length += TaskStatus.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'fleet_manager/QueryFleetAndTasksResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a9cee634e5450b9a2e85c18da8e84744';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 响应部分
    fleet_manager/DroneStatus[] drones
    fleet_manager/TaskStatus[] tasks
    
    
    ================================================================================
    MSG: fleet_manager/DroneStatus
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
    
    ================================================================================
    MSG: fleet_manager/TaskStatus
    # [修改] 在消息定义中添加常量
    # Task Status Constants
    uint8 PENDING=0
    uint8 IN_PROGRESS=1
    uint8 COMPLETED=2
    uint8 NEEDS_BACKUP=3 # 注意：我把FAILED改成了NEEDS_BACKUP以匹配你的C++代码
    
    # Message Fields
    int32 task_id
    nlp_drone_control/Action action_details
    uint8 status
    string assigned_drone_id
    
    ================================================================================
    MSG: nlp_drone_control/Action
    # Represents a single tool call produced by the LLM
    string function_name       # e.g. "takeoff"
    string arguments_json      # raw JSON string of arguments
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new QueryFleetAndTasksResponse(null);
    if (msg.drones !== undefined) {
      resolved.drones = new Array(msg.drones.length);
      for (let i = 0; i < resolved.drones.length; ++i) {
        resolved.drones[i] = DroneStatus.Resolve(msg.drones[i]);
      }
    }
    else {
      resolved.drones = []
    }

    if (msg.tasks !== undefined) {
      resolved.tasks = new Array(msg.tasks.length);
      for (let i = 0; i < resolved.tasks.length; ++i) {
        resolved.tasks[i] = TaskStatus.Resolve(msg.tasks[i]);
      }
    }
    else {
      resolved.tasks = []
    }

    return resolved;
    }
};

module.exports = {
  Request: QueryFleetAndTasksRequest,
  Response: QueryFleetAndTasksResponse,
  md5sum() { return 'a9cee634e5450b9a2e85c18da8e84744'; },
  datatype() { return 'fleet_manager/QueryFleetAndTasks'; }
};
