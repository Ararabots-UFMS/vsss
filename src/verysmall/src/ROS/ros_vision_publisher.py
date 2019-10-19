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

    def publish(self, ball_pos, yellow_pos, yellow_orient, blue_pos, 
                blue_orientation, fps):

        """
            This function publishes in the things position topic

            :param ball_pos: int16[2]
            :param yellow_pos: int16[10]
            :param yellow_orient: int16[5]
            :param blue_pos: int16[10]
            :param blue_orientation: int16[5]
            :param fps: utin16
            :return: returns nothing
        """

        msg = things_position(
            np.int16(ball_pos * 100).tolist(),
            np.int16(yellow_pos.flatten() * 100).tolist(),
            np.int16(yellow_orient.flatten() * 10000).tolist(),
            np.int16(blue_pos.flatten() * 100).tolist(),
            np.int16(blue_orientation.flatten() * 10000).tolist(),
            int(fps * 100)
        )

        try:
            self.pub.publish(msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)
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
