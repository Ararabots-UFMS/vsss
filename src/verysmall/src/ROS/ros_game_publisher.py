#!/usr/bin/env python
import rospy
import math
from verysmall.msg import game_topic
from verysmall.srv import vision_command
from rospy import ServiceException,ServiceProxy, wait_for_service
from random import uniform


class RosGamePublisher:
    """This class can publish Game related messages on a 'Game topic' Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('game', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('game_topic', game_topic, queue_size=1)
        self.msg = game_topic()
        self.msg.robot_roles = '00000'

        # Variable for storing proxy
        self.vision_proxy = None
        self.register_vision_service()

    def set_game_state(self, _game_state):
        self.msg.game_state = _game_state

    def set_robot_role(self, robot_id, role):
        #rospy.logfatal(self.msg.robot_roles)
        self.msg.robot_roles = self.msg.robot_roles[:robot_id] + str(role+1) + self.msg.robot_roles[robot_id+1:]

    def set_penalty_robot(self, _penalty_robot):
        self.msg.penalty_robot = _penalty_robot

    def set_freeball_robot(self, _freeball_robot):
        self.msg.freeball_robot = _freeball_robot

    def set_meta_robot(self, _meta_robot):
        self.msg.meta_robot = _meta_robot

    def set_message(self, game_state, robot_roles, penalty_robot, freeball_robot, meta_robot):
        """
            This function sets the publisher message
            :param game_state: uint8
            :param robot_roles: uint8[5]
            :param penalty_robot: uint8
            :param freeball_robot: uint8
            :param meta_robot: uint8

            :return: returns nothing
        """

        self.msg = game_topic(
            game_state,
            robot_roles,
            penalty_robot,
            freeball_robot,
            meta_robot
        )

    def publish(self):
        """
        This function publishes in the game topic

        :return: returns nothing
        """
        try:
            self.pub.publish(self.msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)

    def register_vision_service(self):
        """Creates a proxy for communicating with service
            :return: returns nothing
        """
        wait_for_service('vision_command')
        self.vision_proxy = ServiceProxy('vision_command', vision_command)

    def send_vision_operation(self, operation):
        """Sends a service request to vision node
            :param operation : uint8

            :return: returns nothing
        """
        wait_for_service('vision_command')
        try:
            self.vision_proxy(operation)
        except ServiceException as exc:
            print("Service did not process request: " + str(exc))

if __name__ == '__main__':
    try:
        RosGamePublisher()
    except rospy.ROSInterruptException:
        pass
