#!/usr/bin/python3
# -*- coding: utf-8 -*-

from utils.json_handler import JsonHandler
from utils.model import Model
from utils.process_killer import ProcessKiller
from interface.Controller.MainWindowController import MainWindowController
from interface.Controller.LoadingController import LoadingController
from ROS.ros_utils import RosUtils
from ROS.ros_game_topic_publisher import GameTopicPublisher
from coach.Coach import Coach
import rospy
from argparse import ArgumentParser
from time import sleep
import roslaunch
from utils.camera_loader import CameraLoader
from random import randint
from sys import argv

"""
Instantiates all the windows, robots, topics and services
"""

if __name__ == '__main__':

    try:
        owner_id = argv[1]
    except ValueError:
        owner_id = 'Player_' + str(randint(0, 99999))

    ProcessKiller(["robot"])

    rospy.init_node('virtual_field', anonymous=True)
    # Load the database
    model = Model()

    # Create roslaunch from API
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    lc = LoadingController()
    lc.start()

    vision_owner = True

    if RosUtils.topic_exists("/things_position"):
        return_type, device_index = -1, -1
        vision_owner = False
    else:
        # Create the GUI
        # Search for the usb camera, if not present, the program ask to a substitute
        # be a file or another camera
        lc.stop()
        return_type, device_index = CameraLoader(model.game_opt['camera']).get_index()

        lc.start("Carregando nó da visão")
        # Launch Vision with another Topic
        arguments = str(device_index) + " " + owner_id

        vision_node = roslaunch.core.Node('verysmall', 'vision_node.py',
                                          name='vision', args=arguments)

        # launches the node and stores it in the given memory space
        vision_process = launch.launch(vision_node)

    game_topic_publisher = GameTopicPublisher(False, model.game_opt, model.robot_params, model.robot_roles,
                                              owner_id)

    coach = Coach(model, game_topic_publisher, launch)

    lc.stop()
    controller = MainWindowController(model, coach, game_topic_publisher)

    lc = LoadingController()
    lc.start("Salvando banco de dados")

    if vision_owner:
        vision_process.stop()

    model.save_params()
    lc.stop()
