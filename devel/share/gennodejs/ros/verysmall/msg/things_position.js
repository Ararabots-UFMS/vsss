// Auto-generated. Do not edit!

// (in-package verysmall.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let five_robot_pos = require('./five_robot_pos.js');
let five_robot_vector = require('./five_robot_vector.js');

//-----------------------------------------------------------

class things_position {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ball_pos = null;
      this.ball_vector = null;
      this.team_pos = null;
      this.team_vector = null;
      this.enemies_pos = null;
      this.enemies_vector = null;
      this.robot_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('ball_pos')) {
        this.ball_pos = initObj.ball_pos
      }
      else {
        this.ball_pos = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('ball_vector')) {
        this.ball_vector = initObj.ball_vector
      }
      else {
        this.ball_vector = new Array(2).fill(0);
      }
      if (initObj.hasOwnProperty('team_pos')) {
        this.team_pos = initObj.team_pos
      }
      else {
        this.team_pos = new five_robot_pos();
      }
      if (initObj.hasOwnProperty('team_vector')) {
        this.team_vector = initObj.team_vector
      }
      else {
        this.team_vector = new five_robot_vector();
      }
      if (initObj.hasOwnProperty('enemies_pos')) {
        this.enemies_pos = initObj.enemies_pos
      }
      else {
        this.enemies_pos = new five_robot_pos();
      }
      if (initObj.hasOwnProperty('enemies_vector')) {
        this.enemies_vector = initObj.enemies_vector
      }
      else {
        this.enemies_vector = new five_robot_vector();
      }
      if (initObj.hasOwnProperty('robot_speed')) {
        this.robot_speed = initObj.robot_speed
      }
      else {
        this.robot_speed = new Array(10).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type things_position
    // Check that the constant length array field [ball_pos] has the right length
    if (obj.ball_pos.length !== 2) {
      throw new Error('Unable to serialize array field ball_pos - length must be 2')
    }
    // Serialize message field [ball_pos]
    bufferOffset = _arraySerializer.uint32(obj.ball_pos, buffer, bufferOffset, 2);
    // Check that the constant length array field [ball_vector] has the right length
    if (obj.ball_vector.length !== 2) {
      throw new Error('Unable to serialize array field ball_vector - length must be 2')
    }
    // Serialize message field [ball_vector]
    bufferOffset = _arraySerializer.float64(obj.ball_vector, buffer, bufferOffset, 2);
    // Serialize message field [team_pos]
    bufferOffset = five_robot_pos.serialize(obj.team_pos, buffer, bufferOffset);
    // Serialize message field [team_vector]
    bufferOffset = five_robot_vector.serialize(obj.team_vector, buffer, bufferOffset);
    // Serialize message field [enemies_pos]
    bufferOffset = five_robot_pos.serialize(obj.enemies_pos, buffer, bufferOffset);
    // Serialize message field [enemies_vector]
    bufferOffset = five_robot_vector.serialize(obj.enemies_vector, buffer, bufferOffset);
    // Check that the constant length array field [robot_speed] has the right length
    if (obj.robot_speed.length !== 10) {
      throw new Error('Unable to serialize array field robot_speed - length must be 10')
    }
    // Serialize message field [robot_speed]
    bufferOffset = _arraySerializer.uint32(obj.robot_speed, buffer, bufferOffset, 10);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type things_position
    let len;
    let data = new things_position(null);
    // Deserialize message field [ball_pos]
    data.ball_pos = _arrayDeserializer.uint32(buffer, bufferOffset, 2)
    // Deserialize message field [ball_vector]
    data.ball_vector = _arrayDeserializer.float64(buffer, bufferOffset, 2)
    // Deserialize message field [team_pos]
    data.team_pos = five_robot_pos.deserialize(buffer, bufferOffset);
    // Deserialize message field [team_vector]
    data.team_vector = five_robot_vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [enemies_pos]
    data.enemies_pos = five_robot_pos.deserialize(buffer, bufferOffset);
    // Deserialize message field [enemies_vector]
    data.enemies_vector = five_robot_vector.deserialize(buffer, bufferOffset);
    // Deserialize message field [robot_speed]
    data.robot_speed = _arrayDeserializer.uint32(buffer, bufferOffset, 10)
    return data;
  }

  static getMessageSize(object) {
    return 224;
  }

  static datatype() {
    // Returns string type for a message object
    return 'verysmall/things_position';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e53313ec081a9c87ca55fbeaff276969';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32[2]		 	        	ball_pos
    float64[2]        				ball_vector
    verysmall/five_robot_pos       	team_pos
    verysmall/five_robot_vector    	team_vector
    verysmall/five_robot_pos       	enemies_pos
    verysmall/five_robot_vector   	enemies_vector
    uint32[10]		 	        	robot_speed
    
    ================================================================================
    MSG: verysmall/five_robot_pos
    uint32[2]    robot_pos_1
    uint32[2]    robot_pos_2
    uint32[2]    robot_pos_3
    uint32[2]    robot_pos_4
    uint32[2]    robot_pos_5
    ================================================================================
    MSG: verysmall/five_robot_vector
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
    const resolved = new things_position(null);
    if (msg.ball_pos !== undefined) {
      resolved.ball_pos = msg.ball_pos;
    }
    else {
      resolved.ball_pos = new Array(2).fill(0)
    }

    if (msg.ball_vector !== undefined) {
      resolved.ball_vector = msg.ball_vector;
    }
    else {
      resolved.ball_vector = new Array(2).fill(0)
    }

    if (msg.team_pos !== undefined) {
      resolved.team_pos = five_robot_pos.Resolve(msg.team_pos)
    }
    else {
      resolved.team_pos = new five_robot_pos()
    }

    if (msg.team_vector !== undefined) {
      resolved.team_vector = five_robot_vector.Resolve(msg.team_vector)
    }
    else {
      resolved.team_vector = new five_robot_vector()
    }

    if (msg.enemies_pos !== undefined) {
      resolved.enemies_pos = five_robot_pos.Resolve(msg.enemies_pos)
    }
    else {
      resolved.enemies_pos = new five_robot_pos()
    }

    if (msg.enemies_vector !== undefined) {
      resolved.enemies_vector = five_robot_vector.Resolve(msg.enemies_vector)
    }
    else {
      resolved.enemies_vector = new five_robot_vector()
    }

    if (msg.robot_speed !== undefined) {
      resolved.robot_speed = msg.robot_speed;
    }
    else {
      resolved.robot_speed = new Array(10).fill(0)
    }

    return resolved;
    }
};

module.exports = things_position;
