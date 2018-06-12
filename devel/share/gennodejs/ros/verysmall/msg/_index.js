
"use strict";

let motor_speed = require('./motor_speed.js');
let five_robot_vector = require('./five_robot_vector.js');
let game_topic = require('./game_topic.js');
let five_robot_pos = require('./five_robot_pos.js');
let things_position = require('./things_position.js');
let comunication_topic = require('./comunication_topic.js');

module.exports = {
  motor_speed: motor_speed,
  five_robot_vector: five_robot_vector,
  game_topic: game_topic,
  five_robot_pos: five_robot_pos,
  things_position: things_position,
  comunication_topic: comunication_topic,
};
