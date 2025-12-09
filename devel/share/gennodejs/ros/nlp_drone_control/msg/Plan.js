// Auto-generated. Do not edit!

// (in-package nlp_drone_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Action = require('./Action.js');

//-----------------------------------------------------------

class Plan {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.actions = null;
      this.replan_mode = null;
    }
    else {
      if (initObj.hasOwnProperty('actions')) {
        this.actions = initObj.actions
      }
      else {
        this.actions = [];
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
    // Serialize message field [actions]
    // Serialize the length for message field [actions]
    bufferOffset = _serializer.uint32(obj.actions.length, buffer, bufferOffset);
    obj.actions.forEach((val) => {
      bufferOffset = Action.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [replan_mode]
    bufferOffset = _serializer.uint8(obj.replan_mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Plan
    let len;
    let data = new Plan(null);
    // Deserialize message field [actions]
    // Deserialize array length for message field [actions]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.actions = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.actions[i] = Action.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [replan_mode]
    data.replan_mode = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.actions.forEach((val) => {
      length += Action.getMessageSize(val);
    });
    return length + 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nlp_drone_control/Plan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13f0666e18573937822408eb65c6a4cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Represents a list of tool calls (a full execution plan)
    nlp_drone_control/Action[] actions
    uint8 replan_mode          # 0 = APPEND, 1 = REPLACE
    
    
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
    const resolved = new Plan(null);
    if (msg.actions !== undefined) {
      resolved.actions = new Array(msg.actions.length);
      for (let i = 0; i < resolved.actions.length; ++i) {
        resolved.actions[i] = Action.Resolve(msg.actions[i]);
      }
    }
    else {
      resolved.actions = []
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
