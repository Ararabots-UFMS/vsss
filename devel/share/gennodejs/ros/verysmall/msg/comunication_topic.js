// Auto-generated. Do not edit!

// (in-package verysmall.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let motor_speed = require('./motor_speed.js');

//-----------------------------------------------------------

class comunication_topic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_id = null;
      this.robots_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_id')) {
        this.robot_id = initObj.robot_id
      }
      else {
        this.robot_id = new Array(10).fill(0);
      }
      if (initObj.hasOwnProperty('robots_speed')) {
        this.robots_speed = initObj.robots_speed
      }
      else {
        this.robots_speed = new Array(10).fill(new motor_speed());
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type comunication_topic
    // Check that the constant length array field [robot_id] has the right length
    if (obj.robot_id.length !== 10) {
      throw new Error('Unable to serialize array field robot_id - length must be 10')
    }
    // Serialize message field [robot_id]
    bufferOffset = _arraySerializer.uint8(obj.robot_id, buffer, bufferOffset, 10);
    // Check that the constant length array field [robots_speed] has the right length
    if (obj.robots_speed.length !== 10) {
      throw new Error('Unable to serialize array field robots_speed - length must be 10')
    }
    // Serialize message field [robots_speed]
    obj.robots_speed.forEach((val) => {
      bufferOffset = motor_speed.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type comunication_topic
    let len;
    let data = new comunication_topic(null);
    // Deserialize message field [robot_id]
    data.robot_id = _arrayDeserializer.uint8(buffer, bufferOffset, 10)
    // Deserialize message field [robots_speed]
    len = 10;
    data.robots_speed = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.robots_speed[i] = motor_speed.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    return 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'verysmall/comunication_topic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bcfc10b8d53657d2a18ba160b07bee7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[10]          robot_id
    motor_speed[10]    robots_speed
    ================================================================================
    MSG: verysmall/motor_speed
    int32            vel_l
    int32            vel_r
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new comunication_topic(null);
    if (msg.robot_id !== undefined) {
      resolved.robot_id = msg.robot_id;
    }
    else {
      resolved.robot_id = new Array(10).fill(0)
    }

    if (msg.robots_speed !== undefined) {
      resolved.robots_speed = new Array(10)
      for (let i = 0; i < resolved.robots_speed.length; ++i) {
        if (msg.robots_speed.length > i) {
          resolved.robots_speed[i] = motor_speed.Resolve(msg.robots_speed[i]);
        }
        else {
          resolved.robots_speed[i] = new motor_speed();
        }
      }
    }
    else {
      resolved.robots_speed = new Array(10).fill(new motor_speed())
    }

    return resolved;
    }
};

module.exports = comunication_topic;
