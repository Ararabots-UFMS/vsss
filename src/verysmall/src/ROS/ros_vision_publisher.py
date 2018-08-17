#!/usr/bin/env python
import rospy
import math
from verysmall.msg import things_position
from verysmall.msg import twofloat64, robot_vector
from random import uniform


class RosVisionPublisher:
    """This class can publish Vision messages on a Things Position Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('vision', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('things_position', things_position, queue_size=1)

        self.empty_robot_vector = .0
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

    def publish(self, ball_pos, ball_speed, team_pos, team_orient, team_speed, enemies_pos, enemies_orientation, enemies_speed):

        """
            This function publishes in the things position topic

            :param ball_pos: 2float64
            :param ball_speed: 2float64
            :param team_pos: 2float64[5]
            :param team_orient: float64[5]
            :param team_speed: 2float64[5]
            :param enemies_pos: 2float64[5]
            :param enemies_orient: 2float64[5]
            :param enemies_speed: 2float64[5]
            :return: returns nothing
        """
        #rospy.logfatal(ball_pos)
        msg = things_position(
            twofloat64(ball_pos[0], ball_pos[1]),
            twofloat64(ball_speed[0], ball_speed[1]),
            [twofloat64(team_pos[0][0], team_pos[0][1]),
             twofloat64(team_pos[1][0], team_pos[1][1]),
             twofloat64(team_pos[2][0], team_pos[2][1]),
             twofloat64(team_pos[3][0], team_pos[3][1]),
             twofloat64(team_pos[4][0], team_pos[4][1])
             ],
            [team_orient[0] if team_orient[0] else self.empty_robot_vector,
             team_orient[1] if team_orient[1] else self.empty_robot_vector,
             team_orient[2] if team_orient[2] else self.empty_robot_vector,
             team_orient[3] if team_orient[3] else self.empty_robot_vector,
             team_orient[4] if team_orient[4] else self.empty_robot_vector
             ],
            [
                twofloat64(team_speed[0][0], team_speed[0][1]),
                twofloat64(team_speed[1][0], team_speed[1][1]),
                twofloat64(team_speed[2][0], team_speed[2][1]),
                twofloat64(team_speed[3][0], team_speed[3][1]),
                twofloat64(team_speed[4][0], team_speed[4][1]),
            ],
            [twofloat64(enemies_pos[0][0], enemies_pos[0][1]),
             twofloat64(enemies_pos[1][0], enemies_pos[1][1]),
             twofloat64(enemies_pos[2][0], enemies_pos[2][1]),
             twofloat64(enemies_pos[3][0], enemies_pos[3][1]),
             twofloat64(enemies_pos[4][0], enemies_pos[4][1])
             ],
            [enemies_orientation[0] if enemies_orientation[0] else self.empty_robot_vector,
             enemies_orientation[1] if enemies_orientation[1] else self.empty_robot_vector,
             enemies_orientation[2] if enemies_orientation[2] else self.empty_robot_vector,
             enemies_orientation[3] if enemies_orientation[3] else self.empty_robot_vector,
             enemies_orientation[4] if enemies_orientation[4] else self.empty_robot_vector
             ],
            [
                twofloat64(enemies_speed[0][0], enemies_speed[0][1]),
                twofloat64(enemies_speed[1][0], enemies_speed[1][1]),
                twofloat64(enemies_speed[2][0], enemies_speed[2][1]),
                twofloat64(enemies_speed[3][0], enemies_speed[3][1]),
                twofloat64(enemies_speed[4][0], enemies_speed[4][1])
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
