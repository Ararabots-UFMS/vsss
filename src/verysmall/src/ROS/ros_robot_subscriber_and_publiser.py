from verysmall.msg import things_position, game_topic, debug_topic
import rospy
import numpy as np
from struct import unpack


class RosRobotSubscriberAndPublisher:
    """
    This class is responsible for reading and formatting ros messages for the robot node
    """
    def __init__(self, _robot, _game_topic_name = 'game_topic_0', _should_debug = False):
        """
        :param _robot: robot object
        """
        rospy.Subscriber('things_position', things_position, self.read_topic , queue_size=10)
        rospy.Subscriber(_game_topic_name, game_topic, self.read_game_topic , queue_size=10)

        if _should_debug:
            rospy.logfatal(_game_topic_name)
            self.pub = rospy.Publisher('debug_topic_'+_game_topic_name.split('_')[2], debug_topic, queue_size=1)

        self.robot = _robot

        self.debug_msg = debug_topic()
        self.debug_msg.id = self.robot.robot_id_integer

    def read_game_topic(self, data):
        """
        Read from game topic callback and open the message into robot variables
        :param data: ROS game topic message
        :return: nothing
        """
        self.robot.game_state = data.game_state
        self.robot.team_side = data.team_side

        #TODO: VERIFICAR ESSA ALTERAÇÃO, N TENHO CERTEZA
        self.robot.role = data.robot_roles[self.robot.robot_id_integer]
        
        self.robot.penalty_robot = data.penalty_robot
        self.robot.freeball_robot = data.freeball_robot
        self.robot.meta_robot = data.meta_robot
        self.robot.changed_game_state = True
        self.robot.state_machine = self.robot.strategies[self.robot.role]
        self.robot.team_color = data.team_color

    def read_topic(self, data) -> None:
        """
        This class formats the things position into np arrays and replaces any nan to None
        :param data: ROS Things position message
        :return: nothing
        """
        self.robot.ball_position = np.array(data.ball_pos) / 100.0
        self.robot.ball_speed = np.array(data.ball_speed) / 100.0

        if(self.robot.team_color == 1): # yellow
            self.robot.team_pos = np.array(data.yellow_team_pos).reshape((-1, 2)) / 100.0
            self.robot.team_orientation = np.array(data.yellow_team_orientation) / 10000.0
            self.robot.team_speed = np.array(data.yellow_team_speed).reshape((-1, 2)) / 100.0
            enemies_position = np.array(data.blue_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(data.blue_team_orientation) / 10000.0
            enemies_speed = np.array(data.blue_team_speed).reshape((-1, 2)) / 100.0
        else: # blue
            self.robot.team_pos = np.array(data.blue_team_pos).reshape((-1, 2)) / 100.0
            self.robot.team_orientation = np.array(data.blue_team_orientation) / 10000.0
            self.robot.team_speed = np.array(data.blue_team_speed).reshape((-1, 2)) / 100.0
            enemies_position = np.array(data.yellow_team_pos).reshape((-1, 2)) / 100.0
            enemies_orientation = np.array(data.yellow_team_orientation) / 10000.0
            enemies_speed = np.array(data.yellow_team_speed).reshape((-1, 2)) / 100.0


        self.robot.position = self.robot.team_pos[self.robot.tag]
        self.robot.orientation = self.robot.team_orientation[self.robot.tag]
        self.robot.speed = self.robot.team_speed[self.robot.tag]

        self.robot.enemies_position = []
        self.robot.enemies_orientation = []
        self.robot.enemies_speed = []

        for i in range(5):
            if np.any(enemies_position[i]):
                self.robot.enemies_position.append(enemies_position[i])
                self.robot.enemies_orientation.append(enemies_orientation[i])
                self.robot.enemies_speed.append(enemies_speed[i])

        self.robot.enemies_position = np.asarray(self.robot.enemies_position)
        self.robot.enemies_orientation = np.asarray(self.robot.enemies_orientation)
        self.robot.enemies_speed = np.asarray(self.robot.enemies_speed)

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
