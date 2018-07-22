#!/usr/bin/python
import sys
from utils.json_handler import JsonHandler
from interface.Controller.MainWindowController import MainWindowController
from ROS.ros_utils import RosUtils
import rospy
import roslaunch
from utils.camera_loader import CameraLoader
"""
Instantiates all the windows, robots, topics and services
"""


class Model():
    """The model class for loading and saving json files"""

    def __init__(self):
        self.json_handler = JsonHandler()
        self.robot_params = self.json_handler.read("parameters/robots.json", escape=True)
        self.robot_pid = self.json_handler.read("parameters/robots_pid.json", escape=True)
        self.robot_bluetooth = self.json_handler.read("parameters/bluetooth.json", escape=True)
        self.robot_roles = self.json_handler.read("parameters/roles.json", escape=True)
        self.game_opt = self.json_handler.read("parameters/game.json", escape=True)
        self.arena_params = "../parameters/ARENA.json"
        self.colors_params = "../parameters/COLORS.json"

    def save_params(self):
        self.json_handler.write(self.robot_params, "parameters/robots.json")
        self.json_handler.write(self.game_opt, "parameters/game.json")


if __name__ == '__main__':
    rospy.init_node('virtual_field', anonymous=True)

    # Load the database
    model = Model()

    if RosUtils.topic_exists("/things_position"):
        print "done"
        return_type, device_index = -1, -1
    else:
        # Create the GUI
        # Search for the usb camera, if not present, the program ask to a substitute
        # be a file or another camera
        return_type, device_index = CameraLoader(model.game_opt['camera']).get_index()

        # Create roslaunch from API
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        #print(str(device_index))
        #sys.exit(-1)

        # Launch Vision with another Topic
        arguments = str(device_index)
        print arguments
        vision_node = roslaunch.core.Node('verysmall', 'vision_node.py',
                                   name='vision', args=arguments)

        # launches the node and stores it in the given memory space
        vision_process = launch.launch(vision_node)

    controller = MainWindowController(model.robot_params, model.robot_bluetooth, model.robot_roles, model.game_opt)

    vision_process.stop()
    model.save_params()
