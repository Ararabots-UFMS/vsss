import rospy
import roslaunch


class RosCoach:
    """
    This class is responsible for creating, modifying and launch robot nodes
    """
    def __init__(self, _launcher):
        """
        :param _launcher: ROSLaunch
        :return: nothing
        """
        if _launcher is None:
            # Create roslaunch from API
            self.launcher = roslaunch.scriptapi.ROSLaunch()
            self.launcher.start()
        else:
            self.launcher = _launcher

        # Allocate robots process
        self.player_process = {}
        self.player_nodes = {}

    def create_and_store_node(self, robot, active, variables):
        """
        Creates a robot node and stores its process for later use
        :param robot: String
        :param active: Bool
        :param variables: String
        :return: nothing
        """
        # creates a node with robot list arguments
        node = roslaunch.core.Node('verysmall', 'robot_node.py',
                                   name=robot,
                                   args=variables)

        # Lets store the node for future alterations
        self.player_nodes[robot] = node

        if active:
            # launches the node and stores it in the given memory space
            self.player_process[robot] = self.launcher.launch(node)
        else:
            self.player_process[robot] = None

    def change_arguments_of_node(self, robot, variables):
        """
        Change arguments from a nope, restart its process in case it is already running
        :param robot: String
        :param variables: String
        :return: nothing
        """
        self.player_nodes[robot].args = variables

        if self.player_process[robot] is None:
            pass
        else:  # We need to restart the node :<
            self.player_process[robot].stop()
            self.player_process[robot] = self.launcher.launch(self.player_nodes[robot])

    def toggle_node_life(self, robot_name, should_be_active):
        """
        Given a robot name, toggle its life
        :param robot_name: String
        :param should_be_active: Bool
        :return: nothing
        """
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

