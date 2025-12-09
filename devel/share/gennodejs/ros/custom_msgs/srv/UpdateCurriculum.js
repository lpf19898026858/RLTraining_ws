// Auto-generated. Do not edit!

// (in-package custom_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class UpdateCurriculumRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.spawn_range_x = null;
      this.spawn_range_z = null;
      this.local_target_range = null;
    }
    else {
      if (initObj.hasOwnProperty('spawn_range_x')) {
        this.spawn_range_x = initObj.spawn_range_x
      }
      else {
        this.spawn_range_x = 0.0;
      }
      if (initObj.hasOwnProperty('spawn_range_z')) {
        this.spawn_range_z = initObj.spawn_range_z
      }
      else {
        this.spawn_range_z = 0.0;
      }
      if (initObj.hasOwnProperty('local_target_range')) {
        this.local_target_range = initObj.local_target_range
      }
      else {
        this.local_target_range = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UpdateCurriculumRequest
    // Serialize message field [spawn_range_x]
    bufferOffset = _serializer.float32(obj.spawn_range_x, buffer, bufferOffset);
    // Serialize message field [spawn_range_z]
    bufferOffset = _serializer.float32(obj.spawn_range_z, buffer, bufferOffset);
    // Serialize message field [local_target_range]
    bufferOffset = _serializer.float32(obj.local_target_range, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateCurriculumRequest
    let len;
    let data = new UpdateCurriculumRequest(null);
    // Deserialize message field [spawn_range_x]
    data.spawn_range_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [spawn_range_z]
    data.spawn_range_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [local_target_range]
    data.local_target_range = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'custom_msgs/UpdateCurriculumRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb353952b83dc48c2837dbeebad71ee4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 spawn_range_x
    float32 spawn_range_z
    float32 local_target_range
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UpdateCurriculumRequest(null);
    if (msg.spawn_range_x !== undefined) {
      resolved.spawn_range_x = msg.spawn_range_x;
    }
    else {
      resolved.spawn_range_x = 0.0
    }

    if (msg.spawn_range_z !== undefined) {
      resolved.spawn_range_z = msg.spawn_range_z;
    }
    else {
      resolved.spawn_range_z = 0.0
    }

    if (msg.local_target_range !== undefined) {
      resolved.local_target_range = msg.local_target_range;
    }
    else {
      resolved.local_target_range = 0.0
    }

    return resolved;
    }
};

class UpdateCurriculumResponse {
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
    // Serializes a message object of type UpdateCurriculumResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UpdateCurriculumResponse
    let len;
    let data = new UpdateCurriculumResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'custom_msgs/UpdateCurriculumResponse';
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
    const resolved = new UpdateCurriculumResponse(null);
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
  Request: UpdateCurriculumRequest,
  Response: UpdateCurriculumResponse,
  md5sum() { return '3b2cd8798f0550ceaa4d103980369410'; },
  datatype() { return 'custom_msgs/UpdateCurriculum'; }
};
