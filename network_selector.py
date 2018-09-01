#!/usr/bin/python
import sys
import os
import fltk as fl
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT'] + "src/"
from interface.Controller.ConnectionController import ConnectionController
from utils.json_handler import JsonHandler

if __name__ == '__main__':
	json_handler = JsonHandler()
	robot_params = json_handler.read(root_path+"parameters/robots.json", escape=True)
	game_opt = json_handler.read(root_path+"parameters/game.json", escape=True)
	window = ConnectionController(robot_params, game_opt)
	window.show()
	fl.Fl.run()
	json_handler.write(game_opt, root_path+"parameters/game.json")