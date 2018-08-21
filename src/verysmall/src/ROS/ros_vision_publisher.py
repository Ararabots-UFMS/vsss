#!/usr/bin/env python
import rospy
#!/usr/bin/env python
PKG = 'verysmall'
import numpy as np
from verysmall.msg import things_position


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

        msg = things_position(
            ball_pos.tolist(),
            ball_speed.tolist(),
            team_pos.flatten().tolist(),
            team_orient.flatten().tolist(),
            team_speed.flatten().tolist(),
            enemies_pos.flatten().tolist(),
            enemies_orientation.flatten().tolist(),
            enemies_speed.flatten().tolist()
        )

        try:
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(msg)


if __name__ == '__main__':
    try:
        RosVisionPublisher()
    except rospy.ROSInterruptException:
        pass
