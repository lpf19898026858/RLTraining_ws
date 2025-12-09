// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DoneInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_done = null;
      this.is_success = null;
      this.reason = null;
    }
    else {
      if (initObj.hasOwnProperty('is_done')) {
        this.is_done = initObj.is_done
      }
      else {
        this.is_done = false;
      }
      if (initObj.hasOwnProperty('is_success')) {
        this.is_success = initObj.is_success
      }
      else {
        this.is_success = false;
      }
      if (initObj.hasOwnProperty('reason')) {
        this.reason = initObj.reason
      }
      else {
        this.reason = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DoneInfo
    // Serialize message field [is_done]
    bufferOffset = _serializer.bool(obj.is_done, buffer, bufferOffset);
    // Serialize message field [is_success]
    bufferOffset = _serializer.bool(obj.is_success, buffer, bufferOffset);
    // Serialize message field [reason]
    bufferOffset = _serializer.string(obj.reason, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DoneInfo
    let len;
    let data = new DoneInfo(null);
    // Deserialize message field [is_done]
    data.is_done = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [is_success]
    data.is_success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [reason]
    data.reason = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.reason);
    return length + 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/DoneInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f554d70bf7374158f003bce0d316e6fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool is_done
    bool is_success
    string reason
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DoneInfo(null);
    if (msg.is_done !== undefined) {
      resolved.is_done = msg.is_done;
    }
    else {
      resolved.is_done = false
    }

    if (msg.is_success !== undefined) {
      resolved.is_success = msg.is_success;
    }
    else {
      resolved.is_success = false
    }

    if (msg.reason !== undefined) {
      resolved.reason = msg.reason;
    }
    else {
      resolved.reason = ''
    }

    return resolved;
    }
};

module.exports = DoneInfo;
