from verysmall.msg import things_position, game_topic, debug_topic
import rospy
import numpy as np
from struct import unpack
from strategy.strategy_utils import GameStates
from strategy.behaviour import Goal


class RosRobotSubscriberAndPublisher:
    """
    This class is responsible for reading and formatting ros messages for the robot node
    """

    def __init__(self, _robot, _game_topic_name='game_topic_0', _should_debug=False):
        """
        :param _robot: robot object
        """
        rospy.Subscriber('things_position', things_position, self.read_topic, queue_size=10)
        rospy.Subscriber(_game_topic_name, game_topic, self.read_game_topic, queue_size=10)

        if _should_debug:
            rospy.logfatal(_game_topic_name)
            self.pub = rospy.Publisher('debug_topic_' + _game_topic_name.split('_')[2], debug_topic, queue_size=1)

        self.robot = _robot

        self.debug_msg = debug_topic()
        self.debug_msg.id = self.robot.id

    def read_game_topic(self, data):
        """
        Read from game topic callback and open the message into robot variables
        :param data: ROS game topic message
        :return: nothing
        """
        self.robot.blackboard.game.state = GameStates(data.game_state)
        self.robot.blackboard.home_goal.side = data.team_side
        self.robot.blackboard.enemy_goal.side = not data.team_side

        self.robot.blackboard.robot.role = data.robot_roles[self.robot.id]

        self.robot.blackboard.game.penalty_robot_id = data.penalty_robot
        self.robot.blackboard.game.freeball_robot_id = data.freeball_robot
        self.robot.blackboard.game.meta_robot_id = data.meta_robot

        self.robot.behaviour_tree = self.robot.behaviour_trees[self.robot.blackboard.robot.role]
        self.robot.team_color = data.team_color

    def read_topic(self, data) -> None:
        """
        This class formats the things position into np arrays and replaces any nan to None
        :param data: ROS Things position message
        :return: nothing
        """
        self.robot.blackboard.ball.position = np.array(data.ball_pos) / 100.0
        self.robot.blackboard.ball.speed = np.array(data.ball_speed) / 100.0

        if self.robot.team_color == 1:  # yellow
            friends_position = np.array(data.yellow_team_pos).reshape((-1, 2)) / 100.0
            friends_orientation = np.array(data.yellow_team_orientation) / 10000.0
            friends_speed = np.array(data.yellow_team_speed).reshape((-1, 2)) / 100.0

            enemies_position = np.array(data.blue_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(data.blue_team_orientation) / 10000.0
            enemies_speed = np.array(data.blue_team_speed).reshape((-1, 2)) / 100.0
        else:  # blue
            friends_position = np.array(data.blue_team_pos).reshape((-1, 2)) / 100.0
            friends_orientation = np.array(data.blue_team_orientation) / 10000.0
            friends_speed = np.array(data.blue_team_speed).reshape((-1, 2)) / 100.0

            enemies_position = np.array(data.yellow_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(data.yellow_team_orientation) / 10000.0
            enemies_speed = np.array(data.yellow_team_speed).reshape((-1, 2)) / 100.0

        self.robot.blackboard.set_robot_variables(friends_position[self.robot.tag],
                                                  friends_speed[self.robot.tag],
                                                  friends_orientation[self.robot.tag])

        self.robot.blackboard.home_team.set_team_variables(friends_position,
                                                           friends_orientation,
                                                           friends_speed)

        self.robot.blackboard.enemy_team.set_team_variables(enemies_position,
                                                            enemies_orientation,
                                                            enemies_speed)
        self.robot.run()

    def debug_publish(self, _vector):

        """
            This function publishes in the debug topic
            :param vector: float64[2]
            :return: returns nothing
        """

        self.debug_msg.vector = _vector

        try:
            self.pub.publish(self.debug_msg)
        except rospy.ROSException as e:
            rospy.logfatal(e)
