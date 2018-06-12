
"use strict";

let motor_speed = require('./motor_speed.js');
let 5_robot_pos = require('./5_robot_pos.js');
let game_topic = require('./game_topic.js');
let things_position = require('./things_position.js');
let comunication_topic = require('./comunication_topic.js');
let 5_robot_vector = require('./5_robot_vector.js');

module.exports = {
  motor_speed: motor_speed,
  5_robot_pos: 5_robot_pos,
  game_topic: game_topic,
  things_position: things_position,
  comunication_topic: comunication_topic,
  5_robot_vector: 5_robot_vector,
};
