import vision_module.sim.packet_pb2 as packet_pb2
import socket
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
        # self.msg.yellow_team_pos[0] = 75
        # self.msg.yellow_team_pos[1] = 75
        # self.ball_pos = [0, 0]
        # self.ball_pos[0] = 5
        # self.ball_pos[1] = 2

        self.message = packet_pb2.Environment()

        UDP_IP = "224.0.0.1"
        UDP_PORT = 10002

        self.sock = socket.socket(socket.AF_INET, # Internet
                            socket.SOCK_DGRAM) # UDP
        
        self.sock.bind((UDP_IP, UDP_PORT))

        # Creates the service responsible for vision modes and operations
        self.service = RosVisionService(self.vision_management)

    def tick(self):

        data, _ = self.sock.recvfrom(1024) # buffer size is 1024 bytes
        #ball.ParseFromString(data)  
        self.message.ParseFromString(data)
        
        pos_offset = 0
        for robot in self.message.frame.robots_yellow:
            self.msg.yellow_team_pos[pos_offset]     = int(robot.x/0.8285*85 + 75)*100
            self.msg.yellow_team_pos[pos_offset + 1] = int(robot.y/0.6285*65 + 65)*100
            self.msg.yellow_team_orientation[int(pos_offset/2)] = int(robot.orientation * 10000)
            pos_offset += 2 

        pos_offset = 0
        for robot in self.message.frame.robots_blue:
            self.msg.blue_team_pos[pos_offset]     = int(robot.x/0.8285*85 + 75)*100
            self.msg.blue_team_pos[pos_offset + 1] = int(robot.y/0.6285*65 + 65)*100
            self.msg.blue_team_orientation[int(pos_offset/2)] = int(robot.orientation * 10000)
            pos_offset += 2 

        self.msg.ball_pos[0] = int(self.message.frame.ball.x/0.8285*85 + 75)*100
        self.msg.ball_pos[1] = int(self.message.frame.ball.y/0.6285 * 65 + 65)*100
        print(self.message.frame.ball.x)
        print(self.message.frame.ball.y)

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
    # rate = rospy.Rate(30)  # 30hz

    while not rospy.is_shutdown():
        vision_node.tick()
        # rate.sleep()