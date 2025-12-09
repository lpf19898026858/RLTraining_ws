// Auto-generated. Do not edit!

// (in-package poi_state_server.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let POIInfo = require('../msg/POIInfo.js');

//-----------------------------------------------------------

class ListPOIsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ListPOIsRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ListPOIsRequest
    let len;
    let data = new ListPOIsRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'poi_state_server/ListPOIsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ListPOIsRequest(null);
    return resolved;
    }
};

class ListPOIsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.pois = null;
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
      if (initObj.hasOwnProperty('pois')) {
        this.pois = initObj.pois
      }
      else {
        this.pois = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ListPOIsResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [pois]
    // Serialize the length for message field [pois]
    bufferOffset = _serializer.uint32(obj.pois.length, buffer, bufferOffset);
    obj.pois.forEach((val) => {
      bufferOffset = POIInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ListPOIsResponse
    let len;
    let data = new ListPOIsResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pois]
    // Deserialize array length for message field [pois]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pois = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pois[i] = POIInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    object.pois.forEach((val) => {
      length += POIInfo.getMessageSize(val);
    });
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'poi_state_server/ListPOIsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5729bcda884e859dbb5f9dba61bed21b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool success
    string message
    poi_state_server/POIInfo[] pois
    
    
    
    ================================================================================
    MSG: poi_state_server/POIInfo
    # POIInfo.msg
    string name
    string type
    string description
    geometry_msgs/Point position
    poi_state_server/NamedPoint[] candidate_points
    geometry_msgs/Point boundary_min
    geometry_msgs/Point boundary_max
    bool has_boundary
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: poi_state_server/NamedPoint
    string name
    geometry_msgs/Point point
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ListPOIsResponse(null);
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

    if (msg.pois !== undefined) {
      resolved.pois = new Array(msg.pois.length);
      for (let i = 0; i < resolved.pois.length; ++i) {
        resolved.pois[i] = POIInfo.Resolve(msg.pois[i]);
      }
    }
    else {
      resolved.pois = []
    }

    return resolved;
    }
};

module.exports = {
  Request: ListPOIsRequest,
  Response: ListPOIsResponse,
  md5sum() { return '5729bcda884e859dbb5f9dba61bed21b'; },
  datatype() { return 'poi_state_server/ListPOIs'; }
};
