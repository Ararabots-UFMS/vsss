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

class game_topic {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_roles = null;
      this.game_state = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_roles')) {
        this.robot_roles = initObj.robot_roles
      }
      else {
        this.robot_roles = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('game_state')) {
        this.game_state = initObj.game_state
      }
      else {
        this.game_state = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type game_topic
    // Check that the constant length array field [robot_roles] has the right length
    if (obj.robot_roles.length !== 3) {
      throw new Error('Unable to serialize array field robot_roles - length must be 3')
    }
    // Serialize message field [robot_roles]
    bufferOffset = _arraySerializer.uint8(obj.robot_roles, buffer, bufferOffset, 3);
    // Serialize message field [game_state]
    bufferOffset = _serializer.uint8(obj.game_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type game_topic
    let len;
    let data = new game_topic(null);
    // Deserialize message field [robot_roles]
    data.robot_roles = _arrayDeserializer.uint8(buffer, bufferOffset, 3)
    // Deserialize message field [game_state]
    data.game_state = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'verysmall/game_topic';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '609f49595bdc94fe57d847d3ac10e1c6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[3]     robot_roles
    uint8        game_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new game_topic(null);
    if (msg.robot_roles !== undefined) {
      resolved.robot_roles = msg.robot_roles;
    }
    else {
      resolved.robot_roles = new Array(3).fill(0)
    }

    if (msg.game_state !== undefined) {
      resolved.game_state = msg.game_state;
    }
    else {
      resolved.game_state = 0
    }

    return resolved;
    }
};

module.exports = game_topic;
