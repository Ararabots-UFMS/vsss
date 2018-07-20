from rospy import get_published_topics


class RosUtils:
    def __init__(self):
        pass

    @staticmethod
    def topic_exists(topic):
        for current_topic in get_published_topics():
            if current_topic == topic:
                return True
        return False
