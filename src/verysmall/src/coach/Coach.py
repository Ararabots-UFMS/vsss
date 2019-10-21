from typing import Dict
from robot_module.robot import Robot
from ROS.ros_coach import RosCoach
from ROS.ros_game_topic_publisher import GameTopicPublisher
from verysmall.msg import game_topic


class Coach:
    """Creates the Coach"""

    def __init__(self, model,
                 _game_topic_publisher: GameTopicPublisher,
                 _launcher=None):

        # Save the parameters for future use
        self.robot_params = model.robot_params
        self.robot_bluetooth = model.robot_bluetooth
        self.robot_roles = model.robot_roles
        self.game_opt = model.game_opt
        self.debug_params = model.debug_params
        self.game_topic_pub = _game_topic_publisher
        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]

        self.ros_functions = RosCoach(_launcher)

        self.create_robots()

    def create_robots(self):
        """
        Reading from database, creates nodes and store them in process
        :return: nothing
        """
        # Doing loops for creating the robot nodes
        for robot in self.robot_params.keys():
            # arguments for the node
            try:
                debug = str(self.debug_params["things"][robot])
                variables = self.get_robot_params(robot,
                                                  self.robot_params[robot],
                                                  self.game_topic_pub.msg,
                                                  int(debug))
            except KeyError:
                variables = ""
                self.robot_params[robot]['active'] = False
            
            suffix = '_' + self.game_topic_pub.get_owner()
            # Creates a player node
            self.ros_functions.create_and_store_node(robot, robot+suffix, self.robot_params[robot]['active'], variables)

    def get_robot_params(self, robot_name: str,
                         robot_params: Dict,
                         topic: game_topic,
                         debug: int = 0) -> str:
        robot_id = int(robot_name.split('_')[1]) - 1
        robot_id = str(robot_id)

        if robot_params["bluetooth_mac_address"] == "nenhum":
            socket_id = str(-1)
        else:
            socket_id = robot_id

        params = ""
        params += robot_id + " "
        params += str(robot_params["tag_number"]) + " "
        params += str(robot_params["body_id"]) + " "
        params += str(topic.team_side) + " "
        params += str(topic.team_color) + " "
        params += str(topic.robot_roles[int(robot_id)]) + " "
        params += self.game_topic_pub.get_owner() + " "
        params += socket_id + " "
        params += str(debug)

        return params

    def change_robot_role(self, robot_id, role):
        """
        This function changes the coach role sugestion and publishes through game_topic
        :param robot_id: int
        :param role: int
        :return: nothing
        """
        # TODO: alterar objeto Coach
        # TODO: alterar funcao para receber string e transformar em int para publicar
        self.game_topic_pub.set_robot_role(robot_id, role)
        self.game_topic_pub.publish()

    def set_robot_parameters(self, robot_id):
        """
        With robot_id, sets all the argument for node
        :param robot_id: int
        :return: nothing
        """
        robot = self.faster_hash[robot_id]

        # arguments for the node
        debug = str(self.debug_params["things"][robot])
        variables = self.get_robot_params(robot,
                                          self.robot_params[robot],
                                          self.game_topic_pub.msg,
                                          debug)

        self.ros_functions.change_arguments_of_node(robot, variables)

    def set_robot_active(self, robot_id, should_be_active):
        """
        Sets a defined robot_id node to start or stop
        :param robot_id: int
        :param should_be_active: bool
        :return: nothing
        """
        robot_name = self.faster_hash[robot_id]
        self.ros_functions.toggle_node_life(robot_name, should_be_active)

    # TODO: Definir run aqui e importala de outro arquivo

    # TODO: Criar pasta para estrategias, paralelo a robot na raiz
