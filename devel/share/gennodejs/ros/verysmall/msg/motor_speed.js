// Auto-generated. Do not edit!

// (in-package verysmall.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class motor_speed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_l = null;
      this.vel_r = null;
    }
    else {
      if (initObj.hasOwnProperty('vel_l')) {
        this.vel_l = initObj.vel_l
      }
      else {
        this.vel_l = 0;
      }
      if (initObj.hasOwnProperty('vel_r')) {
        this.vel_r = initObj.vel_r
      }
      else {
        this.vel_r = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motor_speed
    // Serialize message field [vel_l]
    bufferOffset = _serializer.int32(obj.vel_l, buffer, bufferOffset);
    // Serialize message field [vel_r]
    bufferOffset = _serializer.int32(obj.vel_r, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motor_speed
    let len;
    let data = new motor_speed(null);
    // Deserialize message field [vel_l]
    data.vel_l = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vel_r]
    data.vel_r = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'verysmall/motor_speed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f2dd34999ee8b444344eaa77fc91500c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32            vel_l
    int32            vel_r
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motor_speed(null);
    if (msg.vel_l !== undefined) {
      resolved.vel_l = msg.vel_l;
    }
    else {
      resolved.vel_l = 0
    }

    if (msg.vel_r !== undefined) {
      resolved.vel_r = msg.vel_r;
    }
    else {
      resolved.vel_r = 0
    }

    return resolved;
    }
};

module.exports = motor_speed;
