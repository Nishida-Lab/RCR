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

class AngVel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ang_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('ang_vel')) {
        this.ang_vel = initObj.ang_vel
      }
      else {
        this.ang_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type AngVel
    // Serialize message field [ang_vel]
    bufferOffset = _serializer.float64(obj.ang_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type AngVel
    let len;
    let data = new AngVel(null);
    // Deserialize message field [ang_vel]
    data.ang_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rcr2018/AngVel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0654aa42e34bb45bcf83b5431e81941d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 ang_vel
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new AngVel(null);
    if (msg.ang_vel !== undefined) {
      resolved.ang_vel = msg.ang_vel;
    }
    else {
      resolved.ang_vel = 0.0
    }

    return resolved;
    }
};

module.exports = AngVel;
