// Auto-generated. Do not edit!

// (in-package fleet_manager.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nlp_drone_control = _finder('nlp_drone_control');

//-----------------------------------------------------------

class TaskStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_id = null;
      this.action_details = null;
      this.status = null;
      this.assigned_drone_id = null;
    }
    else {
      if (initObj.hasOwnProperty('task_id')) {
        this.task_id = initObj.task_id
      }
      else {
        this.task_id = 0;
      }
      if (initObj.hasOwnProperty('action_details')) {
        this.action_details = initObj.action_details
      }
      else {
        this.action_details = new nlp_drone_control.msg.Action();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('assigned_drone_id')) {
        this.assigned_drone_id = initObj.assigned_drone_id
      }
      else {
        this.assigned_drone_id = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskStatus
    // Serialize message field [task_id]
    bufferOffset = _serializer.int32(obj.task_id, buffer, bufferOffset);
    // Serialize message field [action_details]
    bufferOffset = nlp_drone_control.msg.Action.serialize(obj.action_details, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [assigned_drone_id]
    bufferOffset = _serializer.string(obj.assigned_drone_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskStatus
    let len;
    let data = new TaskStatus(null);
    // Deserialize message field [task_id]
    data.task_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [action_details]
    data.action_details = nlp_drone_control.msg.Action.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [assigned_drone_id]
    data.assigned_drone_id = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nlp_drone_control.msg.Action.getMessageSize(object.action_details);
    length += _getByteLength(object.assigned_drone_id);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fleet_manager/TaskStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a83d6845a55cd58a28f4d61df9680e1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new TaskStatus(null);
    if (msg.task_id !== undefined) {
      resolved.task_id = msg.task_id;
    }
    else {
      resolved.task_id = 0
    }

    if (msg.action_details !== undefined) {
      resolved.action_details = nlp_drone_control.msg.Action.Resolve(msg.action_details)
    }
    else {
      resolved.action_details = new nlp_drone_control.msg.Action()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.assigned_drone_id !== undefined) {
      resolved.assigned_drone_id = msg.assigned_drone_id;
    }
    else {
      resolved.assigned_drone_id = ''
    }

    return resolved;
    }
};

// Constants for message
TaskStatus.Constants = {
  PENDING: 0,
  IN_PROGRESS: 1,
  COMPLETED: 2,
  NEEDS_BACKUP: 3,
}

module.exports = TaskStatus;
