#!/usr/bin/env python
import rospy
from verysmall.msg import game_topic
from verysmall.srv import vision_command
from rospy import ServiceException,ServiceProxy, wait_for_service


class GameTopicPublisher:
    """
    This class can publish Game related messages on a 'Game topic' Topic
    :return: nothing
    """
    def __init__(self, isnode=False):
        """
        :param isnode: Boolean
        """
        if isnode:  # if this a separeted node
            rospy.init_node('game', anonymous=True)
        # else is only a publisher
        self.pub = rospy.Publisher('game_topic', game_topic, queue_size=1)
        self.msg = game_topic()
        self.msg.robot_roles = [0, 0, 0, 0, 0]

        # Variable for storing proxy
        self.vision_proxy = None
        self.register_vision_service()

    def set_game_state(self, _game_state):
        """
        Sets the running game state
        :param _game_state: int
        :return: nothing
        """
        self.msg.game_state = _game_state

    def set_robot_role(self, robot_id, role):
        """
        Given a robot id, set its role in the game
        :param robot_id: int
        :param role: int
        :return: nothing
        """
        self.msg.robot_roles[robot_id] = role

    def set_penalty_robot(self, _penalty_robot):
        """
        Set the selected robot the action of taking penalty actions
        :param _penalty_robot: int
        :return: nothing
        """
        self.msg.penalty_robot = _penalty_robot

    def set_freeball_robot(self, _freeball_robot):
        self.msg.freeball_robot = _freeball_robot

    def set_meta_robot(self, _meta_robot):
        """
        Set the selected robot the action of taking meta-shot actions
        :param _meta_robot: int
        :return: nothing
        """
        self.msg.meta_robot = _meta_robot

    def set_message(self, game_state, side_of_the_field, robot_roles, penalty_robot, freeball_robot, meta_robot):
        """
        This function sets the publisher message
        :param game_state: uint8
        :param side_of_the_field: uint8
        :param robot_roles: uint8[5]
        :param penalty_robot: uint8
        :param freeball_robot: uint8
        :param meta_robot: uint8

        :return: nothing
        """
        self.msg = game_topic(
            game_state,
            side_of_the_field,
            tuple(robot_roles),
            penalty_robot,
            freeball_robot,
            meta_robot
        )

    def set_side_of_the_field(self, side):
        """
        Set side of the game, right(1) or left(0)?
        :param side: int
        :return: nothing
        """
        self.msg.side_of_the_field = side

    def publish(self):
        """
        This function publishes in the game topic
        :return: nothing
        """
        try:
            self.pub.publish(self.msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)

    def register_vision_service(self):
        """
        Creates a proxy for communicating with service
        :return: nothing
        """
        wait_for_service('vision_command')
        self.vision_proxy = ServiceProxy('vision_command', vision_command)

    def send_vision_operation(self, operation):
        """
        Sends a service request to vision node
        :param operation : uint8
        :return: nothing
        """
        wait_for_service('vision_command')
        try:
            self.vision_proxy(operation)
        except ServiceException as exc:
            print("Service did not process request: " + str(exc))


if __name__ == '__main__':
    try:
        GameTopicPublisher()
    except rospy.ROSInterruptException:
        pass
