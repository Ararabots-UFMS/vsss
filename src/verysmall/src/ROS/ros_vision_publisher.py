#!/usr/bin/env python
import rospy
import math
from verysmall.msg import things_position
from verysmall.msg import robot_pos, robot_vector
from random import uniform


class RosVisionPublisher:
    """This class can publish Vision messages on a Things Position Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('vision', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('things_position', things_position, queue_size=1)

        self.empty_robot_vector = robot_vector()
        self.empty_robot_pos = tuple([.0, .0])

        # example of how to use the virtual_field class
        # Example of publishing a message
        # self.publish(
        #    [0, 0],
        #    0.,
        #    [robot_pos() for _ in range(5)],
        #    [robot_vector() for _ in range(5)],
        #    [home_robot_speed for _ in range(5)]    
        #    [robot_pos() for _ in range(5)],
        #    [robot_vector() for _ in range(5)],

        # )

    def publish(self, ball_pos, ball_speed, team_pos, team_orient, team_speed, enemies_pos, enemies_speed):
        """
            This function publishes in the things position topic

            :param ball_pos: uint32[2]
            :param ball_speed: float64[2]
            :param team_pos: five_robot_pos
            :param team_orient: five_robot_vector
            :param team_speed: [[vx1, vx2], ..., [vx5, vy5]]
            :param enemies_pos: five_robot_pos
            :param enemies_speed: five_robot_vector
            :return: returns nothing
        """

        msg = things_position(
            tuple(ball_pos if all(ball_pos) else [.0, .0]),
            tuple(ball_speed if all(ball_speed) else [.0, .0]),
            [robot_pos(tuple(team_pos[0]) if all(team_pos[0]) else self.empty_robot_pos),
             robot_pos(tuple(team_pos[1]) if all(team_pos[1]) else self.empty_robot_pos),
             robot_pos(tuple(team_pos[2]) if all(team_pos[2]) else self.empty_robot_pos),
             robot_pos(tuple(team_pos[3]) if all(team_pos[3]) else self.empty_robot_pos),
             robot_pos(tuple(team_pos[4]) if all(team_pos[4]) else self.empty_robot_pos)
             ],
            [robot_vector(team_orient[0]) if team_orient[0] else self.empty_robot_vector,
             robot_vector(team_orient[1]) if team_orient[1] else self.empty_robot_vector,
             robot_vector(team_orient[2]) if team_orient[2] else self.empty_robot_vector,
             robot_vector(team_orient[3]) if team_orient[3] else self.empty_robot_vector,
             robot_vector(team_orient[4]) if team_orient[4] else self.empty_robot_vector
             ],
            [robot_pos(tuple(enemies_pos[0]) if all(enemies_pos[0]) else self.empty_robot_pos),
             robot_pos(tuple(enemies_pos[1]) if all(enemies_pos[1]) else self.empty_robot_pos),
             robot_pos(tuple(enemies_pos[2]) if all(enemies_pos[2]) else self.empty_robot_pos),
             robot_pos(tuple(enemies_pos[3]) if all(enemies_pos[3]) else self.empty_robot_pos),
             robot_pos(tuple(enemies_pos[4]) if all(enemies_pos[4]) else self.empty_robot_pos)
             ],
            [self.empty_robot_vector,
             self.empty_robot_vector,
             self.empty_robot_vector,
             self.empty_robot_vector,
             self.empty_robot_vector
             ],
            [robot_pos(tuple(team_speed[0]) if all(team_speed[0]) else  self.empty_robot_pos),
             robot_pos(tuple(team_speed[1]) if all(team_speed[1]) else  self.empty_robot_pos),
             robot_pos(tuple(team_speed[2]) if all(team_speed[2]) else  self.empty_robot_pos),
             robot_pos(tuple(team_speed[3]) if all(team_speed[3]) else  self.empty_robot_pos),
             robot_pos(tuple(team_speed[4]) if all(team_speed[4]) else  self.empty_robot_pos),
             robot_pos(tuple(enemies_speed[0]) if all(enemies_speed[0]) else  self.empty_robot_pos),
             robot_pos(tuple(enemies_speed[1]) if all(enemies_speed[1]) else  self.empty_robot_pos),
             robot_pos(tuple(enemies_speed[2]) if all(enemies_speed[2]) else  self.empty_robot_pos),
             robot_pos(tuple(enemies_speed[3]) if all(enemies_speed[3]) else  self.empty_robot_pos),
             robot_pos(tuple(enemies_speed[4]) if all(enemies_speed[4]) else  self.empty_robot_pos)
             ]
        )


        try:
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)


if __name__ == '__main__':
    try:
        RosVisionPublisher()
    except rospy.ROSInterruptException:
        pass
