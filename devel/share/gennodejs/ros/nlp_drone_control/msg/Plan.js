// Auto-generated. Do not edit!

// (in-package nlp_drone_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ToolCall = require('./ToolCall.js');

//-----------------------------------------------------------

class Plan {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tool_calls = null;
      this.replan_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('tool_calls')) {
        this.tool_calls = initObj.tool_calls
      }
      else {
        this.tool_calls = [];
      }
      if (initObj.hasOwnProperty('replan_mode')) {
        this.replan_mode = initObj.replan_mode
      }
      else {
        this.replan_mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Plan
    // Serialize message field [tool_calls]
    // Serialize the length for message field [tool_calls]
    bufferOffset = _serializer.uint32(obj.tool_calls.length, buffer, bufferOffset);
    obj.tool_calls.forEach((val) => {
      bufferOffset = ToolCall.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [replan_mode]
    bufferOffset = _serializer.uint8(obj.replan_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Plan
    let len;
    let data = new Plan(null);
    // Deserialize message field [tool_calls]
    // Deserialize array length for message field [tool_calls]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.tool_calls = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.tool_calls[i] = ToolCall.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [replan_mode]
    data.replan_mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.tool_calls.forEach((val) => {
      length += ToolCall.getMessageSize(val);
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nlp_drone_control/Plan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6b2a4346febb21fc1800dd9e94fdf8ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Represents a list of tool calls (a full execution plan)
    nlp_drone_control/ToolCall[] tool_calls
    uint8 replan_mode          # 0 = APPEND, 1 = REPLACE
    
    
    ================================================================================
    MSG: nlp_drone_control/ToolCall
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
    const resolved = new Plan(null);
    if (msg.tool_calls !== undefined) {
      resolved.tool_calls = new Array(msg.tool_calls.length);
      for (let i = 0; i < resolved.tool_calls.length; ++i) {
        resolved.tool_calls[i] = ToolCall.Resolve(msg.tool_calls[i]);
      }
    }
    else {
      resolved.tool_calls = []
    }

    if (msg.replan_mode !== undefined) {
      resolved.replan_mode = msg.replan_mode;
    }
    else {
      resolved.replan_mode = 0
    }

    return resolved;
    }
};

module.exports = Plan;
