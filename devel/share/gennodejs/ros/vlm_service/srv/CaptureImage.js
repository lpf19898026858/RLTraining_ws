// Auto-generated. Do not edit!

// (in-package vlm_service.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

class CaptureImageRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.directory_path = null;
      this.file_name = null;
      this.image_topic = null;
    }
    else {
      if (initObj.hasOwnProperty('directory_path')) {
        this.directory_path = initObj.directory_path
      }
      else {
        this.directory_path = '';
      }
      if (initObj.hasOwnProperty('file_name')) {
        this.file_name = initObj.file_name
      }
      else {
        this.file_name = '';
      }
      if (initObj.hasOwnProperty('image_topic')) {
        this.image_topic = initObj.image_topic
      }
      else {
        this.image_topic = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CaptureImageRequest
    // Serialize message field [directory_path]
    bufferOffset = _serializer.string(obj.directory_path, buffer, bufferOffset);
    // Serialize message field [file_name]
    bufferOffset = _serializer.string(obj.file_name, buffer, bufferOffset);
    // Serialize message field [image_topic]
    bufferOffset = _serializer.string(obj.image_topic, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CaptureImageRequest
    let len;
    let data = new CaptureImageRequest(null);
    // Deserialize message field [directory_path]
    data.directory_path = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [file_name]
    data.file_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [image_topic]
    data.image_topic = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.directory_path);
    length += _getByteLength(object.file_name);
    length += _getByteLength(object.image_topic);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vlm_service/CaptureImageRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c143dc1f2dad90abf4369e8bd03fce8b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Request: 指定保存图像的位置和文件名
    string directory_path
    string file_name
    string image_topic  #指定要捕获的图像话题
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CaptureImageRequest(null);
    if (msg.directory_path !== undefined) {
      resolved.directory_path = msg.directory_path;
    }
    else {
      resolved.directory_path = ''
    }

    if (msg.file_name !== undefined) {
      resolved.file_name = msg.file_name;
    }
    else {
      resolved.file_name = ''
    }

    if (msg.image_topic !== undefined) {
      resolved.image_topic = msg.image_topic;
    }
    else {
      resolved.image_topic = ''
    }

    return resolved;
    }
};

class CaptureImageResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.success = null;
      this.message = null;
      this.file_path = null;
      this.captured_image = null;
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
      if (initObj.hasOwnProperty('file_path')) {
        this.file_path = initObj.file_path
      }
      else {
        this.file_path = '';
      }
      if (initObj.hasOwnProperty('captured_image')) {
        this.captured_image = initObj.captured_image
      }
      else {
        this.captured_image = new sensor_msgs.msg.Image();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CaptureImageResponse
    // Serialize message field [success]
    bufferOffset = _serializer.bool(obj.success, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    // Serialize message field [file_path]
    bufferOffset = _serializer.string(obj.file_path, buffer, bufferOffset);
    // Serialize message field [captured_image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.captured_image, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CaptureImageResponse
    let len;
    let data = new CaptureImageResponse(null);
    // Deserialize message field [success]
    data.success = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [file_path]
    data.file_path = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [captured_image]
    data.captured_image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.message);
    length += _getByteLength(object.file_path);
    length += sensor_msgs.msg.Image.getMessageSize(object.captured_image);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'vlm_service/CaptureImageResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '162e78e0b15db8f7728207b46fb9182f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Response: 返回操作结果和保存的完整路径
    bool success
    string message
    string file_path # 返回完整路径，方便客户端（如Unity）确认
    sensor_msgs/Image captured_image  
    
    
    ================================================================================
    MSG: sensor_msgs/Image
    # This message contains an uncompressed image
    # (0, 0) is at top-left corner of image
    #
    
    Header header        # Header timestamp should be acquisition time of image
                         # Header frame_id should be optical frame of camera
                         # origin of frame should be optical center of camera
                         # +x should point to the right in the image
                         # +y should point down in the image
                         # +z should point into to plane of the image
                         # If the frame_id here and the frame_id of the CameraInfo
                         # message associated with the image conflict
                         # the behavior is undefined
    
    uint32 height         # image height, that is, number of rows
    uint32 width          # image width, that is, number of columns
    
    # The legal values for encoding are in file src/image_encodings.cpp
    # If you want to standardize a new string format, join
    # ros-users@lists.sourceforge.net and send an email proposing a new encoding.
    
    string encoding       # Encoding of pixels -- channel meaning, ordering, size
                          # taken from the list of strings in include/sensor_msgs/image_encodings.h
    
    uint8 is_bigendian    # is this data bigendian?
    uint32 step           # Full row length in bytes
    uint8[] data          # actual matrix data, size is (step * rows)
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CaptureImageResponse(null);
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

    if (msg.file_path !== undefined) {
      resolved.file_path = msg.file_path;
    }
    else {
      resolved.file_path = ''
    }

    if (msg.captured_image !== undefined) {
      resolved.captured_image = sensor_msgs.msg.Image.Resolve(msg.captured_image)
    }
    else {
      resolved.captured_image = new sensor_msgs.msg.Image()
    }

    return resolved;
    }
};

module.exports = {
  Request: CaptureImageRequest,
  Response: CaptureImageResponse,
  md5sum() { return 'e3c78bc34cba5b744044b3a476675c16'; },
  datatype() { return 'vlm_service/CaptureImage'; }
};
