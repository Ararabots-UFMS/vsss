#!/usr/bin/env python
import rospy
import math
from verysmall.msg import game_topic
from random import uniform


class RosGamePublisher:
    """This class can publish Game related messages on a 'Game topic' Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('game', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('game_topic', game_topic, queue_size=1)

    def publish(self, game_state, robot_roles, penalty_robot, freeball_robot, meta_robot):
        """
            This function publishes in the game topic
            :param game_state: uint8
            :param robot_roles: uint8[5]
            :param penalty_robot: uint8
            :param freeball_robot: uint8
            :param meta_robot: uint8

            :return: returns nothing
        """

        msg = game_topic(
            game_state,
            robot_roles,
            penalty_robot,
            freeball_robot,
            meta_robot
        )

        try:
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)


if __name__ == '__main__':
    try:
        RosGamePublisher()
    except rospy.ROSInterruptException:
        pass
