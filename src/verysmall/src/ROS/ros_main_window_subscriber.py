#!/usr/bin/env python
import rospy
from queue import Queue
from collections import deque
from verysmall.msg import things_position, debug_topic
from numpy import nan_to_num, array, asarray, any


class RosMainWindowSubscriber:
    def __init__(self, _game_topic_name = 'game_topic_0'):
        """
        This is class is mainly responsible for the ROS functions of Main Window View
        :return: nothing
        """
        #if isnode:
        #    rospy.init_node('virtual_field', anonymous=True)

        # Ros node for reading the buffer
        rospy.Subscriber('things_position', things_position, self.read, queue_size=10)

        # Debug topic
        rospy.Subscriber('debug_topic_'+_game_topic_name.split('_')[2], debug_topic, self.read_debug_topic, queue_size= 10)

        # Queue of data from Topic Things position
        msg = things_position()
        debug_msg = debug_topic()

        self.data = deque([msg], maxlen = 60)  # Shapes the size of the Queue
        self.debug_data = deque([debug_msg], maxlen = 10)

    def read_debug_topic(self, debug_data):
        """
        Read from topic callback and appends to data buffer
        :return: nothing
        """
        # Inserts data in the Queue
        self.debug_data.append(debug_data)

    def read(self, data):
        """
        Read from topic callback and appends to data buffer
        :return: nothing
        """
        # Inserts data in the Queue
        self.data.append(data)

    def pop_item_debug(self):
        """
        Grabs an message from the queue and returns it in np.array format
        :return data_item : things position message
        """
        debug_item = None

        debug_item = [-1,-1]
        
        try:
            debug_item = self.debug_data.popleft()
            debug_item = [debug_item.id, nan_to_num(array(debug_item.vector))]
        
        except IndexError:
            rospy.loginfo("vazia")

        return debug_item


    def pop_item(self):
        """
        Grabs an message from the queue and returns it in np.array format
        :return data_item : things position message
        """
        data_item = None

        try:
            data_item = self.data.popleft()
            data_item.ball_pos = nan_to_num(data_item.ball_pos)  # ball position
            data_item.team_pos = nan_to_num(data_item.team_pos).reshape((5, 2))  # home team position
            data_item.team_orientation = nan_to_num(data_item.team_orientation)  # home team vectors
            data_item.team_speed = nan_to_num(data_item.team_speed).reshape((5, 2))  # away team speed    
            
            data_item.enemies_pos = nan_to_num(data_item.enemies_pos).reshape((5, 2))  # away team position
            data_item.enemies_orientation = nan_to_num(data_item.enemies_orientation)  # away team vectors
            data_item.enemies_speed = nan_to_num(data_item.enemies_speed).reshape((5, 2))  # away team speed

            enemies_position = []
            enemies_orientation = []
            enemies_speed = []

            for i in xrange(5):
                if any(data_item.enemies_pos[i]):
                    enemies_position.append(data_item.enemies_pos[i])
                    enemies_orientation.append(data_item.enemies_orientation[i])
                    enemies_speed.append(data_item.enemies_speed[i])

            data_item.enemies_pos = asarray(enemies_position)
            data_item.enemies_orientation = asarray(enemies_orientation)
            data_item.enemies_speed = asarray(enemies_speed)


        except IndexError:
            rospy.loginfo("vazia")

        return data_item
