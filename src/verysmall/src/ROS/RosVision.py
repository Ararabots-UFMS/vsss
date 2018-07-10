#!/usr/bin/env python
import rospy
import math
from verysmall.msg import things_position
from verysmall.msg import robot_pos, robot_vector
from random import uniform


class RosVision:
    """This class can publish Vision messages on a Things Position Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('vision', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('things_position', things_position, queue_size=1)

        # example of how to use the virtual_field class
        # Example of publishing a message
        # self.publish(
        #    [0, 0],
        #    0.,
        #    [robot_pos() for _ in range(5)],
        #    [robot_vector() for _ in range(5)],
        #    [robot_pos() for _ in range(5)],
        #    [robot_vector() for _ in range(5)],
        #    [0] * 10
        # )

    def publish(self, ball_pos, ball_vector, team_pos, team_vector, enemies_pos, enemies_vector, robot_speed):
        """
            This function publishes in the things position topic

            :param ball_pos: uint32[2]
            :param ball_vector: float64[2]
            :param team_pos: five_robot_pos
            :param team_vector: five_robot_vector
            :param enemies_pos: five_robot_pos
            :param enemies_vector: five_robot_vector
            :param robot_speed: uint32[10]
            :return: returns nothing
        """
        msg = things_position(
            ball_pos, ball_vector,
            team_pos, team_vector,
            enemies_pos, enemies_vector,
            robot_speed
        )
        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        RosVision()
    except rospy.ROSInterruptException:
        pass
