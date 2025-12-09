// Auto-generated. Do not edit!

// (in-package nlp_drone_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ToolCall {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.function_name = null;
      this.arguments_json = null;
    }
    else {
      if (initObj.hasOwnProperty('function_name')) {
        this.function_name = initObj.function_name
      }
      else {
        this.function_name = '';
      }
      if (initObj.hasOwnProperty('arguments_json')) {
        this.arguments_json = initObj.arguments_json
      }
      else {
        this.arguments_json = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ToolCall
    // Serialize message field [function_name]
    bufferOffset = _serializer.string(obj.function_name, buffer, bufferOffset);
    // Serialize message field [arguments_json]
    bufferOffset = _serializer.string(obj.arguments_json, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ToolCall
    let len;
    let data = new ToolCall(null);
    // Deserialize message field [function_name]
    data.function_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [arguments_json]
    data.arguments_json = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.function_name);
    length += _getByteLength(object.arguments_json);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'nlp_drone_control/ToolCall';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '45955b02be53f69e3133212fb3d15576';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new ToolCall(null);
    if (msg.function_name !== undefined) {
      resolved.function_name = msg.function_name;
    }
    else {
      resolved.function_name = ''
    }

    if (msg.arguments_json !== undefined) {
      resolved.arguments_json = msg.arguments_json;
    }
    else {
      resolved.arguments_json = ''
    }

    return resolved;
    }
};

module.exports = ToolCall;
