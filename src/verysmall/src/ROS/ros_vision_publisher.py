#!/usr/bin/env python
import rospy
#!/usr/bin/env python
PKG = 'verysmall'
import numpy as np
from verysmall.msg import things_position
from verysmall.srv import vision_command


class RosVisionService:
    """
    This Class implements a service for changing vision parameters
    """
    def __init__(self, _request_function):
        """
        :param _request_function: function callback
        """
        self.core = rospy.Service('vision_command', vision_command, _request_function)


class RosVisionPublisher:
    """This class can publish Vision messages on a Things Position Topic"""
    def __init__(self, isnode=False):
        if isnode:  # if this a separeted node
            rospy.init_node('vision', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('things_position', things_position, queue_size=1)

    def publish(self, ball_pos, ball_speed, team_pos =np.array([]), team_orient =np.array([]), team_speed =np.array([]),
                enemies_pos =np.array([]), enemies_orientation =np.array([]), enemies_speed =np.array([]), fps=0.0):

        """
            This function publishes in the things position topic

            :param ball_pos: float64[2]
            :param ball_speed: float64[2]
            :param team_pos: float64[10]
            :param team_orient: float64[5]
            :param team_speed: float64[10]
            :param enemies_pos: float64[10]
            :param enemies_orientation: float64[5]
            :param enemies_speed: float64[10]
            :param fps: float
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
            enemies_speed.flatten().tolist(),
            fps
        )

        try:
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(msg)


if __name__ == '__main__':
    try:
        r = RosVisionPublisher(True)
        msg = things_position()
        msg.ball_pos = (100.0,100.0)
        msg.team_pos[0] = 50.0
        msg.team_pos[1] = 50.0
        while not rospy.is_shutdown():
            r.pub.publish(msg)
    except rospy.ROSInterruptException:
        pass
