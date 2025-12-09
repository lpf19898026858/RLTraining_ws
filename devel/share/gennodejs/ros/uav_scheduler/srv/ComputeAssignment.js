// Auto-generated. Do not edit!

// (in-package uav_scheduler.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ComputeAssignmentRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.algorithm = null;
      this.drones = null;
      this.pois = null;
    }
    else {
      if (initObj.hasOwnProperty('algorithm')) {
        this.algorithm = initObj.algorithm
      }
      else {
        this.algorithm = '';
      }
      if (initObj.hasOwnProperty('drones')) {
        this.drones = initObj.drones
      }
      else {
        this.drones = [];
      }
      if (initObj.hasOwnProperty('pois')) {
        this.pois = initObj.pois
      }
      else {
        this.pois = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeAssignmentRequest
    // Serialize message field [algorithm]
    bufferOffset = _serializer.string(obj.algorithm, buffer, bufferOffset);
    // Serialize message field [drones]
    bufferOffset = _arraySerializer.string(obj.drones, buffer, bufferOffset, null);
    // Serialize message field [pois]
    bufferOffset = _arraySerializer.string(obj.pois, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeAssignmentRequest
    let len;
    let data = new ComputeAssignmentRequest(null);
    // Deserialize message field [algorithm]
    data.algorithm = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [drones]
    data.drones = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [pois]
    data.pois = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.algorithm);
    object.drones.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    object.pois.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_scheduler/ComputeAssignmentRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c8cce96eb27806832f78ff0d0d73a350';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 输入
    string algorithm              # "hungarian", "auction", "genetic", "distributed_auction"
    string[] drones
    string[] pois
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeAssignmentRequest(null);
    if (msg.algorithm !== undefined) {
      resolved.algorithm = msg.algorithm;
    }
    else {
      resolved.algorithm = ''
    }

    if (msg.drones !== undefined) {
      resolved.drones = msg.drones;
    }
    else {
      resolved.drones = []
    }

    if (msg.pois !== undefined) {
      resolved.pois = msg.pois;
    }
    else {
      resolved.pois = []
    }

    return resolved;
    }
};

class ComputeAssignmentResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.assignments = null;
      this.plan_text = null;
    }
    else {
      if (initObj.hasOwnProperty('assignments')) {
        this.assignments = initObj.assignments
      }
      else {
        this.assignments = [];
      }
      if (initObj.hasOwnProperty('plan_text')) {
        this.plan_text = initObj.plan_text
      }
      else {
        this.plan_text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ComputeAssignmentResponse
    // Serialize message field [assignments]
    bufferOffset = _arraySerializer.string(obj.assignments, buffer, bufferOffset, null);
    // Serialize message field [plan_text]
    bufferOffset = _serializer.string(obj.plan_text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ComputeAssignmentResponse
    let len;
    let data = new ComputeAssignmentResponse(null);
    // Deserialize message field [assignments]
    data.assignments = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [plan_text]
    data.plan_text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.assignments.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += _getByteLength(object.plan_text);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'uav_scheduler/ComputeAssignmentResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e46fb8d1fa4812f772ec00b1475a3a6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 输出
    string[] assignments          # 匹配结果，例如 V_UAV_0->House1
    string plan_text              # 任务动作文本
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ComputeAssignmentResponse(null);
    if (msg.assignments !== undefined) {
      resolved.assignments = msg.assignments;
    }
    else {
      resolved.assignments = []
    }

    if (msg.plan_text !== undefined) {
      resolved.plan_text = msg.plan_text;
    }
    else {
      resolved.plan_text = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ComputeAssignmentRequest,
  Response: ComputeAssignmentResponse,
  md5sum() { return '97370e26157faf720c13585eba94765b'; },
  datatype() { return 'uav_scheduler/ComputeAssignment'; }
};
