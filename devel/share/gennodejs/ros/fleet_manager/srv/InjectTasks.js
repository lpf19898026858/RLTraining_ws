// Auto-generated. Do not edit!

// (in-package fleet_manager.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TaskStatus = require('../msg/TaskStatus.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class InjectTasksRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tasks = null;
      this.assign_to_drone_id = null;
    }
    else {
      if (initObj.hasOwnProperty('tasks')) {
        this.tasks = initObj.tasks
      }
      else {
        this.tasks = [];
      }
      if (initObj.hasOwnProperty('assign_to_drone_id')) {
        this.assign_to_drone_id = initObj.assign_to_drone_id
      }
      else {
        this.assign_to_drone_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InjectTasksRequest
    // Serialize message field [tasks]
    // Serialize the length for message field [tasks]
    bufferOffset = _serializer.uint32(obj.tasks.length, buffer, bufferOffset);
    obj.tasks.forEach((val) => {
      bufferOffset = TaskStatus.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [assign_to_drone_id]
    bufferOffset = _serializer.string(obj.assign_to_drone_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InjectTasksRequest
    let len;
    let data = new InjectTasksRequest(null);
    // Deserialize message field [tasks]
    // Deserialize array length for message field [tasks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tasks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tasks[i] = TaskStatus.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [assign_to_drone_id]
    data.assign_to_drone_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.tasks.forEach((val) => {
      length += TaskStatus.getMessageSize(val);
    });
    length += _getByteLength(object.assign_to_drone_id);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'fleet_manager/InjectTasksRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a336e44c7ab89d7dbd3e266b33615a37';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    fleet_manager/TaskStatus[] tasks
    string assign_to_drone_id
    
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
    const resolved = new InjectTasksRequest(null);
    if (msg.tasks !== undefined) {
      resolved.tasks = new Array(msg.tasks.length);
      for (let i = 0; i < resolved.tasks.length; ++i) {
        resolved.tasks[i] = TaskStatus.Resolve(msg.tasks[i]);
      }
    }
    else {
      resolved.tasks = []
    }

    if (msg.assign_to_drone_id !== undefined) {
      resolved.assign_to_drone_id = msg.assign_to_drone_id;
    }
    else {
      resolved.assign_to_drone_id = ''
    }

    return resolved;
    }
};

class InjectTasksResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type InjectTasksResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type InjectTasksResponse
    let len;
    let data = new InjectTasksResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'fleet_manager/InjectTasksResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '358e233cde0c8a8bcfea4ce193f8fc15';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new InjectTasksResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    return resolved;
    }
};

module.exports = {
  Request: InjectTasksRequest,
  Response: InjectTasksResponse,
  md5sum() { return '72ac7b4be33eb67b134d27e55502c210'; },
  datatype() { return 'fleet_manager/InjectTasks'; }
};
