from robot.robot import Robot
from verysmall.srv import manage_mac
import rospy
import roslaunch


class Team():
    """Crates all the robots and Coach"""

    def send_mac_address_operation(self, operation, robot_id, mac):
        # Register or Remove a bluetooth device associated with a robot
        rospy.wait_for_service('manage_mac')
        try:
            self.bluetooth_proxy(operation, robot_id, mac)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def register_mac_service(self):
        # Creates a proxy for communicating with service
        rospy.wait_for_service('manage_mac')
        self.bluetooth_proxy = rospy.ServiceProxy('manage_mac', manage_mac)

    def __init__(self, robot_list):
        # Create roslaunch from API
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()

        # Creates the bluetooth node and launch using a process
        bluetooth_node = roslaunch.core.Node('verysmall', 'bluetooth_sender.py', name='bluetooth_sender')
        self.bluetooth_process = launch.launch(bluetooth_node)

        # Allocate proxy and robots process
        self.bluetooth_proxy = None
        self.player_process = [None] * 5

        # Doing two loops. one for creating the nodes
        # another for updating the mac address list
        for robot_id in robot_list:
            # arguments for the node
            variables = str(robot_id) + ' ' + robot_list[robot_id][1]

            # creates a node with robot list arguments
            node = roslaunch.core.Node('verysmall', 'robot_node.py',
                                       name='robot_' + str(robot_id),
                                       args=variables)
            # launches the node and stores it in the given memory space
            self.player_process[robot_id - 1] = launch.launch(node)

        # calls the service register function
        self.register_mac_service()

        # for each robot in the team
        # register its mac on the bluetooth node list
        for robot_id in robot_list:
            self.send_mac_address_operation(1, robot_id, robot_list[robot_id][0])


if __name__ == '__main__':
    rospy.init_node('team', anonymous=False)
    team_list = {1: ('20:1a:06:ad:ba:90', 'bd_1'), 2: ('20:1a:06:ad:ba:90', 'bd_2'), 3: ('20:1a:06:ad:ba:90', 'bd_3')}
    Team(team_list)
    rospy.spin()
