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
    def __init__(self, isnode=False, _game_opt = None, _robot_params = None, _robot_name_roles = None, topic_number = 0):
        """
        :param isnode: Boolean
        :param _game_opt: Game json
        :param _robot_params: Robots Json
        :param _robot_name_roles: Robot roles Json
        :param topic_number: int
        """
        if isnode:  # if this a separeted node
            rospy.init_node('game', anonymous=True)

        # else is only a publisher
        self.name = 'game_topic'+str(topic_number)

        self.pub = rospy.Publisher(self.name, game_topic, queue_size=1)
        self.msg = game_topic()
        self.msg.robot_roles = [0, 0, 0, 0, 0]
        
        self.game_opt = _game_opt
        self.robot_params = _robot_params
        self.robot_name_roles = _robot_name_roles

        # Init message values
        self.faster_hash = ['robot_'+str(x) for x in range(1, 6)]

        for robot_id in xrange(5):
            role_name = self.robot_params[self.faster_hash[robot_id]]['role']
            self.set_robot_role(robot_id, self.robot_name_roles[role_name])

        self.set_freeball_robot(self.game_opt['freeball_player'])
        self.set_penalty_robot(self.game_opt['penalty_player'])
        self.set_meta_robot(self.game_opt['meta_player']) 

        self.set_team_side(self.game_opt['side'])

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

    def assigned_action_to_robot(self, action_id, robot_id):
        """
        Assigned a robot_id to action(meta, free or penalty)
            - 0 -> Freeball
            - 1 -> Penalti
            - 2 -> Tiro de meta
        :param action_id: int
        :param robot_id: int
        :return: nothing
        """
        if action_id:
            if action_id == 1:
                self.set_penalty_robot(robot_id)
            else:
                self.set_meta_robot(robot_id)
        else:
            self.set_freeball_robot(robot_id)

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

    def set_message(self, game_state, team_side, robot_roles, penalty_robot, freeball_robot, meta_robot):
        """
        This function sets the publisher message
        :param game_state: uint8
        :param team_side: uint8
        :param robot_roles: uint8[5]
        :param penalty_robot: uint8
        :param freeball_robot: uint8
        :param meta_robot: uint8

        :return: nothing
        """
        self.msg = game_topic(
            game_state,
            team_side,
            tuple(robot_roles),
            penalty_robot,
            freeball_robot,
            meta_robot
        )

    def set_team_side(self, side):
        """
        Set side of the game, right(1) or left(0)?
        :param side: int
        :return: nothing
        """
        self.msg.team_side = side

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
