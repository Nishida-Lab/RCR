// Auto-generated. Do not edit!

// (in-package rcr2018.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TofSide {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.right = null;
      this.left = null;
    }
    else {
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = 0.0;
      }
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TofSide
    // Serialize message field [right]
    bufferOffset = _serializer.float64(obj.right, buffer, bufferOffset);
    // Serialize message field [left]
    bufferOffset = _serializer.float64(obj.left, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TofSide
    let len;
    let data = new TofSide(null);
    // Deserialize message field [right]
    data.right = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [left]
    data.left = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rcr2018/TofSide';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '69961745475ae50d2f0b8ca4d0df065b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 right
    float64 left
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TofSide(null);
    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = 0.0
    }

    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = 0.0
    }

    return resolved;
    }
};

module.exports = TofSide;
