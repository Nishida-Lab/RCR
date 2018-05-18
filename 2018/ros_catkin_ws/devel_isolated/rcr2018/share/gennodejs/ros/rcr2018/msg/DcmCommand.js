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

class DcmCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd_vel')) {
        this.cmd_vel = initObj.cmd_vel
      }
      else {
        this.cmd_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DcmCommand
    // Serialize message field [cmd_vel]
    bufferOffset = _serializer.float64(obj.cmd_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DcmCommand
    let len;
    let data = new DcmCommand(null);
    // Deserialize message field [cmd_vel]
    data.cmd_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rcr2018/DcmCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '93e14900f3ac3cbfca813e5ff5e2bd6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 cmd_vel
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DcmCommand(null);
    if (msg.cmd_vel !== undefined) {
      resolved.cmd_vel = msg.cmd_vel;
    }
    else {
      resolved.cmd_vel = 0.0
    }

    return resolved;
    }
};

module.exports = DcmCommand;
