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

class SvmCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd_ang_vel = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd_ang_vel')) {
        this.cmd_ang_vel = initObj.cmd_ang_vel
      }
      else {
        this.cmd_ang_vel = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SvmCommand
    // Serialize message field [cmd_ang_vel]
    bufferOffset = _serializer.float64(obj.cmd_ang_vel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SvmCommand
    let len;
    let data = new SvmCommand(null);
    // Deserialize message field [cmd_ang_vel]
    data.cmd_ang_vel = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rcr2018/SvmCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6dddac8b1d4dfc1d29704413439e983';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 cmd_ang_vel
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SvmCommand(null);
    if (msg.cmd_ang_vel !== undefined) {
      resolved.cmd_ang_vel = msg.cmd_ang_vel;
    }
    else {
      resolved.cmd_ang_vel = 0.0
    }

    return resolved;
    }
};

module.exports = SvmCommand;
