import rospy
import sys
import cv2

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
        self.msg.yellow_team_pos[0] = 75
        self.msg.yellow_team_pos[1] = 75
        self.msg.ball_pos[0] = 5
        self.msg.ball_pos[1] = 2
        self.state = 0

        # Creates the service responsible for vision modes and operations
        self.service = RosVisionService(self.vision_management)

    def tick(self):

        if self.state == 0:
            self.msg.ball_pos[0] += 1
            if self.msg.ball_pos[0] == 145:
                self.state = 1
        elif self.state == 1:
            self.msg.ball_pos[1]+= 1
            if self.msg.ball_pos[1] == 130:
                self.state = 2
        elif self.state == 2:
            self.msg.ball_pos[0]-=1
            if self.msg.ball_pos[0] == 5:
                self.state = 3
        else:
            self.msg.ball_pos[1]-=1
            if self.msg.ball_pos[1] == 2:
                self.state = 0 

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
    rate = rospy.Rate(30)  # 30hz

    while not rospy.is_shutdown():
        vision_node.tick()
        rate.sleep()
