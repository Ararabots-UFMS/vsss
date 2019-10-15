import rospy
from queue import Queue
from collections import deque, namedtuple
import numpy as np
from numpy import array, asarray, any
from verysmall.msg import things_position, debug_topic
#from interface.View.MainWindowView import MainWindowView

class RosMainWindowSubscriber:
    def __init__(self, window, game_topic_name = 'game_topic_0'):
        """
        This is class is mainly responsible for the ROS functions of Main Window View
        :return: nothing
        """
        self._my_window = window
        #if isnode:
        #    rospy.init_node('virtual_field', anonymous=True)

        # Ros node for reading the buffer
        rospy.Subscriber('things_position', things_position, self.read, queue_size=5)

        # Debug topic
        rospy.Subscriber('debug_topic_'+game_topic_name.split('_')[2], debug_topic, self.read_debug_topic, queue_size= 5)

        # Queue of data from Topic Things position
        msg = things_position()
        debug_msg = debug_topic()

        self.data = deque([msg], maxlen = 5)  # Shapes the size of the Queue
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
            topic_info = self.data.popleft()
            data_item = namedtuple('DataItem', ['ball_pos', 'team_pos', 
                                   'team_orientation', 'team_speed',
                                   'enemies_pos', 'enemies_orientation',
                                   'enemies_speed', 'vision_fps'])
            
            data_item.ball_pos = np.array(topic_info.ball_pos) / 100.0
            data_item.vision_fps = topic_info.vision_fps / 100.0

            if self._my_window.home_color == 1:
                data_item.team_pos = np.array(topic_info.yellow_team_pos).reshape((5, 2)) / 100.0
                data_item.team_orientation = np.array(topic_info.yellow_team_orientation) / 10000.0
                data_item.team_speed = np.array(topic_info.yellow_team_speed).reshape((5, 2)) / 100.0
                data_item.enemies_pos = np.array(topic_info.blue_team_pos).reshape((5, 2)) / 100.0
                data_item.enemies_orientation = np.array(topic_info.blue_team_orientation) / 10000.0
                data_item.enemies_speed = np.array(topic_info.blue_team_speed).reshape((5, 2)) / 100.0
            else:
                data_item.team_pos = np.array(topic_info.blue_team_pos).reshape((5, 2)) / 100.0
                data_item.team_orientation = np.array(topic_info.blue_team_orientation) / 10000.0
                data_item.team_speed = np.array(topic_info.blue_team_speed).reshape((5, 2)) / 100.0
                data_item.enemies_pos = np.array(topic_info.yellow_team_pos).reshape((5, 2)) / 100.0
                data_item.enemies_orientation = np.array(topic_info.yellow_team_orientation) / 10000.0
                data_item.enemies_speed = np.array(topic_info.yellow_team_speed).reshape((5, 2)) / 100.0


            enemies_position = []
            enemies_orientation = []
            enemies_speed = []

            for i in range(5):
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
