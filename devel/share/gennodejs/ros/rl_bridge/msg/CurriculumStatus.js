// Auto-generated. Do not edit!

// (in-package rl_bridge.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CurriculumStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.lesson_id = null;
      this.lesson_name = null;
      this.changed = null;
    }
    else {
      if (initObj.hasOwnProperty('lesson_id')) {
        this.lesson_id = initObj.lesson_id
      }
      else {
        this.lesson_id = 0;
      }
      if (initObj.hasOwnProperty('lesson_name')) {
        this.lesson_name = initObj.lesson_name
      }
      else {
        this.lesson_name = '';
      }
      if (initObj.hasOwnProperty('changed')) {
        this.changed = initObj.changed
      }
      else {
        this.changed = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CurriculumStatus
    // Serialize message field [lesson_id]
    bufferOffset = _serializer.int32(obj.lesson_id, buffer, bufferOffset);
    // Serialize message field [lesson_name]
    bufferOffset = _serializer.string(obj.lesson_name, buffer, bufferOffset);
    // Serialize message field [changed]
    bufferOffset = _serializer.bool(obj.changed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CurriculumStatus
    let len;
    let data = new CurriculumStatus(null);
    // Deserialize message field [lesson_id]
    data.lesson_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [lesson_name]
    data.lesson_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [changed]
    data.changed = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.lesson_name);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rl_bridge/CurriculumStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eed77d69c41fbfca0645d6c4f9105016';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int32 lesson_id
    string lesson_name
    bool changed
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CurriculumStatus(null);
    if (msg.lesson_id !== undefined) {
      resolved.lesson_id = msg.lesson_id;
    }
    else {
      resolved.lesson_id = 0
    }

    if (msg.lesson_name !== undefined) {
      resolved.lesson_name = msg.lesson_name;
    }
    else {
      resolved.lesson_name = ''
    }

    if (msg.changed !== undefined) {
      resolved.changed = msg.changed;
    }
    else {
      resolved.changed = false
    }

    return resolved;
    }
};

module.exports = CurriculumStatus;
