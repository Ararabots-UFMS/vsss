from rospy import get_published_topics
import random

class RosUtils:
    def __init__(self):
        pass

    @staticmethod
    def topic_exists(topic):
        """
        Returns if topic is already live
        :param topic: String
        :return: bool
        """
        for owner_topics in get_published_topics():
            for current_topic in owner_topics:
                if current_topic == topic:
                    return True
        return False

    @staticmethod
    def number_of_topic_instances(topic):
        """
        Returns the number of instances of a topic, given a prefix
        e.g: game_topic returns 3 when:
            - game_topic_0
            - game_topic_1
            - game_topic_2
        :param topic: String
        :return: int
        """        
        return random.randint(0,99999)
