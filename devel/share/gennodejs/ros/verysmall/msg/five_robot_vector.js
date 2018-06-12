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

class five_robot_vector {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.robot_angle_vector_1 = null;
      this.robot_angle_vector_2 = null;
      this.robot_angle_vector_3 = null;
      this.robot_angle_vector_4 = null;
      this.robot_angle_vector_5 = null;
    }
    else {
      if (initObj.hasOwnProperty('robot_angle_vector_1')) {
        this.robot_angle_vector_1 = initObj.robot_angle_vector_1
      }
      else {
        this.robot_angle_vector_1 = 0.0;
      }
      if (initObj.hasOwnProperty('robot_angle_vector_2')) {
        this.robot_angle_vector_2 = initObj.robot_angle_vector_2
      }
      else {
        this.robot_angle_vector_2 = 0.0;
      }
      if (initObj.hasOwnProperty('robot_angle_vector_3')) {
        this.robot_angle_vector_3 = initObj.robot_angle_vector_3
      }
      else {
        this.robot_angle_vector_3 = 0.0;
      }
      if (initObj.hasOwnProperty('robot_angle_vector_4')) {
        this.robot_angle_vector_4 = initObj.robot_angle_vector_4
      }
      else {
        this.robot_angle_vector_4 = 0.0;
      }
      if (initObj.hasOwnProperty('robot_angle_vector_5')) {
        this.robot_angle_vector_5 = initObj.robot_angle_vector_5
      }
      else {
        this.robot_angle_vector_5 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type five_robot_vector
    // Serialize message field [robot_angle_vector_1]
    bufferOffset = _serializer.float64(obj.robot_angle_vector_1, buffer, bufferOffset);
    // Serialize message field [robot_angle_vector_2]
    bufferOffset = _serializer.float64(obj.robot_angle_vector_2, buffer, bufferOffset);
    // Serialize message field [robot_angle_vector_3]
    bufferOffset = _serializer.float64(obj.robot_angle_vector_3, buffer, bufferOffset);
    // Serialize message field [robot_angle_vector_4]
    bufferOffset = _serializer.float64(obj.robot_angle_vector_4, buffer, bufferOffset);
    // Serialize message field [robot_angle_vector_5]
    bufferOffset = _serializer.float64(obj.robot_angle_vector_5, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type five_robot_vector
    let len;
    let data = new five_robot_vector(null);
    // Deserialize message field [robot_angle_vector_1]
    data.robot_angle_vector_1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_angle_vector_2]
    data.robot_angle_vector_2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_angle_vector_3]
    data.robot_angle_vector_3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_angle_vector_4]
    data.robot_angle_vector_4 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_angle_vector_5]
    data.robot_angle_vector_5 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'verysmall/five_robot_vector';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '46d2ecd06a68bed7c3b95711f5808fee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64   robot_angle_vector_1
    float64   robot_angle_vector_2
    float64   robot_angle_vector_3
    float64   robot_angle_vector_4
    float64   robot_angle_vector_5
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new five_robot_vector(null);
    if (msg.robot_angle_vector_1 !== undefined) {
      resolved.robot_angle_vector_1 = msg.robot_angle_vector_1;
    }
    else {
      resolved.robot_angle_vector_1 = 0.0
    }

    if (msg.robot_angle_vector_2 !== undefined) {
      resolved.robot_angle_vector_2 = msg.robot_angle_vector_2;
    }
    else {
      resolved.robot_angle_vector_2 = 0.0
    }

    if (msg.robot_angle_vector_3 !== undefined) {
      resolved.robot_angle_vector_3 = msg.robot_angle_vector_3;
    }
    else {
      resolved.robot_angle_vector_3 = 0.0
    }

    if (msg.robot_angle_vector_4 !== undefined) {
      resolved.robot_angle_vector_4 = msg.robot_angle_vector_4;
    }
    else {
      resolved.robot_angle_vector_4 = 0.0
    }

    if (msg.robot_angle_vector_5 !== undefined) {
      resolved.robot_angle_vector_5 = msg.robot_angle_vector_5;
    }
    else {
      resolved.robot_angle_vector_5 = 0.0
    }

    return resolved;
    }
};

module.exports = five_robot_vector;
