#!/usr/bin/env python
import rospy
#!/usr/bin/env python
PKG = 'verysmall'
import roslib; roslib.load_manifest(PKG)
from rospy.numpy_msg import numpy_msg
import numpy as np
from verysmall.msg import things_position
from verysmall.msg import twofloat64, robot_vector



class RosVisionPublisher:
    """This class can publish Vision messages on a Things Position Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('vision', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('things_position', (things_position), queue_size=1)

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
            ball_pos,
            ball_speed,
            team_pos.flatten(),
            team_orient.flatten(),
            team_speed.flatten(),
            enemies_pos.flatten(),
            enemies_orientation.flatten(),
            enemies_speed.flatten()
        )
        rospy.logfatal(team_pos[0])
        try:
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)


if __name__ == '__main__':
    try:
        RosVisionPublisher()
    except rospy.ROSInterruptException:
        pass
