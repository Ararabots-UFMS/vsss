#!/usr/bin/python
# -*- coding: latin-1 -*-
from utils.json_handler import JsonHandler
from utils.model import Model
from interface.Controller.MainWindowController import MainWindowController
from interface.Controller.LoadingController import LoadingController
from ROS.ros_utils import RosUtils
from trainer import Trainer
import rospy
import roslaunch
from utils.camera_loader import CameraLoader
"""
Instantiates all the windows, robots, topics and services
"""

if __name__ == '__main__':
    rospy.init_node('virtual_field', anonymous=True)

    # Load the database
    model = Model()

    # Create roslaunch from API
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    lc = LoadingController()
    lc.start("Carregando Assets")

    if RosUtils.topic_exists("/things_position"):
        return_type, device_index = -1, -1
        vision_owner = False
    else:
        # Create the GUI
        # Search for the usb camera, if not present, the program ask to a substitute
        # be a file or another camera
        lc.stop()
        return_type, device_index = CameraLoader(model.game_opt['camera']).get_index()
        lc.start("Carregando n� da vis�o")
        # Launch Vision with another Topic
        arguments = str(device_index)

        vision_node = roslaunch.core.Node('verysmall', 'vision_node.py',
                                          name='vision', args=arguments)

        # launches the node and stores it in the given memory space
        vision_process = launch.launch(vision_node)
        vision_owner = True

    trainer = Trainer(model.robot_params, model.robot_bluetooth, model.robot_roles, launch)
    lc.stop()
    controller = MainWindowController(model.robot_params, model.robot_bluetooth, model.robot_roles, model.game_opt,
                                      trainer)
    lc.start("Salvando banco de dados")
    if vision_owner:
        vision_process.stop()

    model.save_params()
    lc.stop()