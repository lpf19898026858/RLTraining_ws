// Auto-generated. Do not edit!

// (in-package nlp_drone_control.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ExecuteDroneActionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_type = null;
      this.params_json = null;
    }
    else {
      if (initObj.hasOwnProperty('action_type')) {
        this.action_type = initObj.action_type
      }
      else {
        this.action_type = '';
      }
      if (initObj.hasOwnProperty('params_json')) {
        this.params_json = initObj.params_json
      }
      else {
        this.params_json = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExecuteDroneActionRequest
    // Serialize message field [action_type]
    bufferOffset = _serializer.string(obj.action_type, buffer, bufferOffset);
    // Serialize message field [params_json]
    bufferOffset = _serializer.string(obj.params_json, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteDroneActionRequest
    let len;
    let data = new ExecuteDroneActionRequest(null);
    // Deserialize message field [action_type]
    data.action_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [params_json]
    data.params_json = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.action_type);
    length += _getByteLength(object.params_json);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'nlp_drone_control/ExecuteDroneActionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2601bc24fd4babc166c150f2f3f63219';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # --- Request ---
    # 动作的类型，例如 "adjust_camera", "rotate_360", "hover_and_monitor"
    string action_type
    
    # (可选) 动作的参数，我们可以用一个JSON字符串来灵活地传递
    # 例如：对于adjust_camera, params可以是 "{\"pitch_angle\": -90.0}"
    # 对于hover_and_monitor, params可以是 "{\"duration_seconds\": 5.0}"
    string params_json
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExecuteDroneActionRequest(null);
    if (msg.action_type !== undefined) {
      resolved.action_type = msg.action_type;
    }
    else {
      resolved.action_type = ''
    }

    if (msg.params_json !== undefined) {
      resolved.params_json = msg.params_json;
    }
    else {
      resolved.params_json = ''
    }

    return resolved;
    }
};

class ExecuteDroneActionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('success')) {
        this.success = initObj.success
      }
      else {
        this.success = false;
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ExecuteDroneActionResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ExecuteDroneActionResponse
    let len;
    let data = new ExecuteDroneActionResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'nlp_drone_control/ExecuteDroneActionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '937c9679a518e3a18d831e57125ea522';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # --- Response ---
    bool success          # 动作是否成功执行
    string message        # 返回给LLM的信息，例如 "Camera adjusted." 或 "Rotation failed."
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ExecuteDroneActionResponse(null);
    if (msg.success !== undefined) {
      resolved.success = msg.success;
    }
    else {
      resolved.success = false
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: ExecuteDroneActionRequest,
  Response: ExecuteDroneActionResponse,
  md5sum() { return '77584cf99b7cb35fa13143d3f49ac104'; },
  datatype() { return 'nlp_drone_control/ExecuteDroneAction'; }
};
