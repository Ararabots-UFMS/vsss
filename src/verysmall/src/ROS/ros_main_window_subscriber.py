#!/usr/bin/env python
import rospy
from Queue import Queue
from collections import deque
from verysmall.msg import things_position
from numpy import nan_to_num, array


class RosMainWindowSubscriber:
    def __init__(self, isnode=True):
        """
        This is class is mainly responsible for the ROS functions of Main Window View
        :return: nothing
        """
        if isnode:
            rospy.init_node('virtual_field', anonymous=True)

        # Ros node for reading the buffer
        rospy.Subscriber('things_position', things_position, self.read, queue_size=1)

        # Queue of data from Topic Things position
        msg = things_position()
        self.data = deque([msg, msg])  # Shapes the size of the Queue

    def read(self, data):
        """
        Read from topic callback and appends to data buffer
        :return: nothing
        """
        # Inserts data in the Queue
        self.data.append(data)

    def pop_item(self):
        """
        Grabs an message from the queue and returns it in np.array format
        :return data_item : things position message
        """
        data_item = None

        try:
            data_item = self.data.popleft()
            data_item.ball_pos = nan_to_num(array(data_item.ball_pos))  # ball position
            data_item.team_pos = nan_to_num(array(data_item.team_pos)).reshape((5, 2))  # home team position
            data_item.team_orientation = nan_to_num(array(data_item.team_orientation))  # home team vectors
            data_item.team_speed = nan_to_num(array(data_item.team_speed)).reshape((5, 2))  # away team speed    
            data_item.enemies_pos = nan_to_num(data_item.enemies_pos).reshape((5, 2))  # away team position
            data_item.enemies_orientation = nan_to_num(data_item.enemies_orientation)  # away team vectors
            data_item.enemies_speed = nan_to_num(array(data_item.enemies_speed)).reshape((5, 2))  # away team speed
        except IndexError:
            rospy.loginfo("vazia")

        return data_item
