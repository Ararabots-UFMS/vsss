#!/usr/bin/python
import rospy
import sys
import cv2
from camera.camera import Camera
from threading import Thread
from vision import Vision
from time import time
import os
from enum import Enum
old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']+"src/"
from ROS.ros_vision_publisher import RosVisionPublisher
from ROS.ros_vision_publisher import RosVisionService
sys.path[0] = old_path

from verysmall.msg import things_position

class VisionOperations(Enum):
    """
    This class stores vision operations for service request
    """
    SHOW = 1
    CROPPER = 2
    COLOR_CALIBRATION = 3
    SET_TEAM_COLOR_BLUE = 4
    SET_TEAM_COLOR_YELLOW = 5

class VisionNode:
    """
    A node for spinning the Vision
    """
    def __init__(self):

        self.mercury = RosVisionPublisher(True)
        self.msg = things_position()
        self.msg.ball_pos = [75,75]
        self.msg.team_pos[0] = 0
        self.msg.team_pos[1] = 0


        # Creates the service responsible for vision modes and operations
        self.service = RosVisionService(self.vision_management)

    def tick(self):
        self.msg.team_pos[0] = (self.msg.team_pos[0] + 1)%120
        self.msg.team_pos[1] = (self.msg.team_pos[1] + 1)%120
        self.msg.team_pos[2] = (self.msg.team_pos[2] + 1)%120
        self.msg.team_pos[3] = 120
        self.mercury.pub.publish(self.msg)


    def vision_management(self, req):
        """
        This is the reading function for a service response
        :param req: variable to get the request operation
        :return: bool
        """
        success = True
        self.state_changed = req.operation
        return success


if __name__ == "__main__":

    vision_node = VisionNode()
    rate = rospy.Rate(10)  # 30hz

    while not rospy.is_shutdown():
        vision_node.tick()
        rate.sleep()
