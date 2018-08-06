from rospy import get_published_topics


class RosUtils:
    def __init__(self):
        pass

    @staticmethod
    def topic_exists(topic):
        for owner_topics in get_published_topics():
            for current_topic in owner_topics:
                if current_topic == topic:
                    return True
        return False
