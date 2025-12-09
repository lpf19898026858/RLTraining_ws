// Auto-generated. Do not edit!

// (in-package poi_state_server.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NamedPoint = require('./NamedPoint.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class POIInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.name = null;
      this.type = null;
      this.description = null;
      this.position = null;
      this.candidate_points = null;
      this.boundary_min = null;
      this.boundary_max = null;
      this.has_boundary = null;
    }
    else {
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = '';
      }
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('candidate_points')) {
        this.candidate_points = initObj.candidate_points
      }
      else {
        this.candidate_points = [];
      }
      if (initObj.hasOwnProperty('boundary_min')) {
        this.boundary_min = initObj.boundary_min
      }
      else {
        this.boundary_min = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('boundary_max')) {
        this.boundary_max = initObj.boundary_max
      }
      else {
        this.boundary_max = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('has_boundary')) {
        this.has_boundary = initObj.has_boundary
      }
      else {
        this.has_boundary = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type POIInfo
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.string(obj.type, buffer, bufferOffset);
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [candidate_points]
    // Serialize the length for message field [candidate_points]
    bufferOffset = _serializer.uint32(obj.candidate_points.length, buffer, bufferOffset);
    obj.candidate_points.forEach((val) => {
      bufferOffset = NamedPoint.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [boundary_min]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.boundary_min, buffer, bufferOffset);
    // Serialize message field [boundary_max]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.boundary_max, buffer, bufferOffset);
    // Serialize message field [has_boundary]
    bufferOffset = _serializer.bool(obj.has_boundary, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type POIInfo
    let len;
    let data = new POIInfo(null);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [candidate_points]
    // Deserialize array length for message field [candidate_points]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.candidate_points = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.candidate_points[i] = NamedPoint.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [boundary_min]
    data.boundary_min = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [boundary_max]
    data.boundary_max = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [has_boundary]
    data.has_boundary = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.type);
    length += _getByteLength(object.description);
    object.candidate_points.forEach((val) => {
      length += NamedPoint.getMessageSize(val);
    });
    return length + 89;
  }

  static datatype() {
    // Returns string type for a message object
    return 'poi_state_server/POIInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b1a999c05b4e892b655e03546fef00e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new POIInfo(null);
    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = ''
    }

    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.candidate_points !== undefined) {
      resolved.candidate_points = new Array(msg.candidate_points.length);
      for (let i = 0; i < resolved.candidate_points.length; ++i) {
        resolved.candidate_points[i] = NamedPoint.Resolve(msg.candidate_points[i]);
      }
    }
    else {
      resolved.candidate_points = []
    }

    if (msg.boundary_min !== undefined) {
      resolved.boundary_min = geometry_msgs.msg.Point.Resolve(msg.boundary_min)
    }
    else {
      resolved.boundary_min = new geometry_msgs.msg.Point()
    }

    if (msg.boundary_max !== undefined) {
      resolved.boundary_max = geometry_msgs.msg.Point.Resolve(msg.boundary_max)
    }
    else {
      resolved.boundary_max = new geometry_msgs.msg.Point()
    }

    if (msg.has_boundary !== undefined) {
      resolved.has_boundary = msg.has_boundary;
    }
    else {
      resolved.has_boundary = false
    }

    return resolved;
    }
};

module.exports = POIInfo;
