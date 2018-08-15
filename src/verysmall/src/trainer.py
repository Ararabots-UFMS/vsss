#!/usr/bin/python
from robot.robot import Robot
from verysmall.srv import manage_mac
import rospy
import roslaunch


class Trainer:
    """Creates the Trainer"""

    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _game_opt, _launcher=None,):

        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.game_opt = _game_opt

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]

        if _launcher is None:
            # Create roslaunch from API
            self.launcher = roslaunch.scriptapi.ROSLaunch()
            self.launcher.start()
        else:
            self.launcher = _launcher

        # Allocate robots process
        self.player_process = {}
        self.player_nodes = {}

        #TODO: Criar funcao para instanciar robos

        # Doing loops for creating the robot nodes
        for robot in self.robot_params.keys():
            # arguments for the node
            bluetooth_number = self.robot_params[robot]['bluetooth_mac_address']
            variables = robot + ' ' + self.robot_bluetooth[bluetooth_number] + " " + self.robot_params[robot]['body_id']

            # creates a node with robot list arguments
            node = roslaunch.core.Node('verysmall', 'robot_node.py',
                                       name=robot,
                                       args=variables)

            # Lets store the node for future alterations
            self.player_nodes[robot] = node

            if self.robot_params[robot]['active']:
                # launches the node and stores it in the given memory space
                self.player_process[robot] = self.launcher.launch(node)
            else:
                self.player_process[robot] = None

    def set_robot_bluetooth(self, robot_id):
        robot = self.faster_hash[robot_id]

        # arguments for the node
        bluetooth_number = self.robot_params[robot]['bluetooth_mac_address']
        variables = robot + ' ' + self.robot_bluetooth[bluetooth_number] + " " + self.robot_params[robot]['body_id']
        self.player_nodes[robot].args = variables

        if self.player_process[robot] is None:
            pass
        else:  # We need to restart the node :<
            self.player_process[robot].stop()
            self.player_process[robot] = self.launcher.launch(self.player_nodes[robot])

    def set_robot_active(self, robot_id, should_be_active):
        robot_name = self.faster_hash[robot_id]

        if should_be_active:  # This robot should be active
            if self.player_process[robot_name] is None:  # Missing the process
                self.player_process[robot_name] = self.launcher.launch(self.player_nodes[robot_name])
            elif not self.player_process[robot_name].is_alive():  # Must start process first
                self.player_process[robot_name].start()
            else:  # Do nothing if process is ok
                pass
        else:  # I dont want you anymore
            if self.player_process[robot_name] is None:
                pass  # Do nothing since is already dead
            elif not self.player_process[robot_name].is_alive():
                self.player_process[robot_name] = None
            else:  # Process is alive and well, so lets kill him >:D
                self.player_process[robot_name].stop()
                self.player_process[robot_name] = None

    #TODO: Tirar set_robot_role do publisher e adiciona-lo aqui

    #TODO: Passar arquivo de trainer para propria pasta coach. bem como o nome da classe

    #TODO: Definir run aqui e importala de outro arquivo

    #TODO: Criar pasta para estrategias, paralelo a robot na raiz