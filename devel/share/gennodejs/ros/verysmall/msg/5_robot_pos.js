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

class 5_robot_pos {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_pos_1 = null;
      this.robot_pos_2 = null;
      this.robot_pos_3 = null;
      this.robot_pos_4 = null;
      this.robot_pos_5 = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_pos_1')) {
        this.robot_pos_1 = initObj.robot_pos_1
      }
      else {
        this.robot_pos_1 = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('robot_pos_2')) {
        this.robot_pos_2 = initObj.robot_pos_2
      }
      else {
        this.robot_pos_2 = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('robot_pos_3')) {
        this.robot_pos_3 = initObj.robot_pos_3
      }
      else {
        this.robot_pos_3 = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('robot_pos_4')) {
        this.robot_pos_4 = initObj.robot_pos_4
      }
      else {
        this.robot_pos_4 = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('robot_pos_5')) {
        this.robot_pos_5 = initObj.robot_pos_5
      }
      else {
        this.robot_pos_5 = new Array(2).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type 5_robot_pos
    // Check that the constant length array field [robot_pos_1] has the right length
    if (obj.robot_pos_1.length !== 2) {
      throw new Error('Unable to serialize array field robot_pos_1 - length must be 2')
    }
    // Serialize message field [robot_pos_1]
    bufferOffset = _arraySerializer.uint32(obj.robot_pos_1, buffer, bufferOffset, 2);
    // Check that the constant length array field [robot_pos_2] has the right length
    if (obj.robot_pos_2.length !== 2) {
      throw new Error('Unable to serialize array field robot_pos_2 - length must be 2')
    }
    // Serialize message field [robot_pos_2]
    bufferOffset = _arraySerializer.uint32(obj.robot_pos_2, buffer, bufferOffset, 2);
    // Check that the constant length array field [robot_pos_3] has the right length
    if (obj.robot_pos_3.length !== 2) {
      throw new Error('Unable to serialize array field robot_pos_3 - length must be 2')
    }
    // Serialize message field [robot_pos_3]
    bufferOffset = _arraySerializer.uint32(obj.robot_pos_3, buffer, bufferOffset, 2);
    // Check that the constant length array field [robot_pos_4] has the right length
    if (obj.robot_pos_4.length !== 2) {
      throw new Error('Unable to serialize array field robot_pos_4 - length must be 2')
    }
    // Serialize message field [robot_pos_4]
    bufferOffset = _arraySerializer.uint32(obj.robot_pos_4, buffer, bufferOffset, 2);
    // Check that the constant length array field [robot_pos_5] has the right length
    if (obj.robot_pos_5.length !== 2) {
      throw new Error('Unable to serialize array field robot_pos_5 - length must be 2')
    }
    // Serialize message field [robot_pos_5]
    bufferOffset = _arraySerializer.uint32(obj.robot_pos_5, buffer, bufferOffset, 2);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type 5_robot_pos
    let len;
    let data = new 5_robot_pos(null);
    // Deserialize message field [robot_pos_1]
    data.robot_pos_1 = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    // Deserialize message field [robot_pos_2]
    data.robot_pos_2 = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    // Deserialize message field [robot_pos_3]
    data.robot_pos_3 = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    // Deserialize message field [robot_pos_4]
    data.robot_pos_4 = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    // Deserialize message field [robot_pos_5]
    data.robot_pos_5 = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'verysmall/5_robot_pos';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2fb152bde934739c2e0092eef499be2d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32[2]    robot_pos_1
    uint32[2]    robot_pos_2
    uint32[2]    robot_pos_3
    uint32[2]    robot_pos_4
    uint32[2]    robot_pos_5
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new 5_robot_pos(null);
    if (msg.robot_pos_1 !== undefined) {
      resolved.robot_pos_1 = msg.robot_pos_1;
    }
    else {
      resolved.robot_pos_1 = new Array(2).fill(0)
    }

    if (msg.robot_pos_2 !== undefined) {
      resolved.robot_pos_2 = msg.robot_pos_2;
    }
    else {
      resolved.robot_pos_2 = new Array(2).fill(0)
    }

    if (msg.robot_pos_3 !== undefined) {
      resolved.robot_pos_3 = msg.robot_pos_3;
    }
    else {
      resolved.robot_pos_3 = new Array(2).fill(0)
    }

    if (msg.robot_pos_4 !== undefined) {
      resolved.robot_pos_4 = msg.robot_pos_4;
    }
    else {
      resolved.robot_pos_4 = new Array(2).fill(0)
    }

    if (msg.robot_pos_5 !== undefined) {
      resolved.robot_pos_5 = msg.robot_pos_5;
    }
    else {
      resolved.robot_pos_5 = new Array(2).fill(0)
    }

    return resolved;
    }
};

module.exports = 5_robot_pos;
