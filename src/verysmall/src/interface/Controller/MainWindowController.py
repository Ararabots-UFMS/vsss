from ..View.MainWindowView import MainWindowView
from BluetoothManagerController import BluetoothManagerController
from ConnectionController import ConnectionController
from DebugController import DebugController
import fltk as fl
import sys
import os
from verysmall.msg import game_topic

old_path = sys.path[0]
sys.path[0] = root_path = os.environ['ROS_ARARA_ROOT']
from verysmall.srv import vision_command

sys.path[0] +="src/"
from ROS.ros_game_publisher import RosGamePublisher
from rospy import ServiceException,ServiceProxy, wait_for_service
sys.path[0] = old_path

class MainWindowController():
    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _game_opt, _debug_params, _trainer):

        # The controllers are created but not show
        self.bluetooth_controller = BluetoothManagerController(_robot_bluetooth, hidden=True)
        self.connection_controller = ConnectionController(_robot_bluetooth, _game_opt)
        self.debug_controller = DebugController(_debug_params, hidden=True)

        # Lets create the view of our controller shall we
        self.view = MainWindowView()

        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.game_opt = _game_opt

        # The trainer object class control the active and the activities of robots
        self.trainer = _trainer

        # Since our primary keys are the keys of the dict it self
        # and since our DataBase is simple, it can be stored as simple strings
        self.robot_roles_keys = sorted(self.robot_roles.keys())
        self.robot_bluetooth_keys = self.robot_bluetooth.keys()

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_'+str(x) for x in range(1, 6)]
        self.assigned_robot_text = ["Jogador "+str(x) for x in range(1, 6)]
        self.assigned_robot_indexes = ['penalty_player', 'freeball_player', 'meta_player']
        self.view.team_color.value(self.game_opt["time"])
        self.view.team_side.value(self.game_opt["side"])

        # Creates the game topic
        self.pub = RosGamePublisher()

        # Variable for storing proxy
        self.vision_proxy = None
        self.register_mac_service()

        #replaced by the function set_robots_bluetooth
        self.set_robots_bluetooth()


        # A loop for the assigned robot actions
        for num in range(3):

            # Same idea here with the value indexes
            current_item = 0
            for item in self.assigned_robot_text:
                # Add the name of player to the drop-down...
                self.view.option_robots[num].add(item)

                # If robot name is equal to the game action, set as the current value
                if self.game_opt[self.assigned_robot_indexes[num]] == current_item:
                    # Same as above
                    self.view.option_robots[num].value(current_item)
                # Well...
                current_item += 1

            # Set a callback for input and button
            self.view.option_robots[num].callback(self.action_input_choice)
            self.view.action_buttons[num].callback(self.action_button_clicked)
        # Multiple callbacks, each for one type of input
        # but since whe have ids for each robot input
        # we can parse through each using its on dictionary
        for num in xrange(self.view.n_robots):
            self.view.robot_bluetooths[num].callback(self.bluetooth_choice)
            self.view.robot_roles[num].callback(self.role_choice)
            self.view.robot_radio_button[num].callback(self.radio_choice)
            
        self.view.top_menu.callback(self.top_menu_choice)
        self.view.play_button.callback(self.action_button_clicked)

        self.view.team_color.callback(self.on_color_change)
        self.view.team_side.callback(self.on_side_change)

        self.view.end()

    def set_robots_bluetooth(self):
        # For each Robot, this loop covers all the inputs
        for num in xrange(self.view.n_robots):
            # Access the robot params dict using a int
            # and stores its reference in a temporary variable
            current_robot = self.robot_params[self.faster_hash[num]]

            # This integer is for the value of item in the
            # Drop-down choice box

            #"Nenhum" means the zero value, in case the bluetooth name of the robot changes
            self.view.robot_bluetooths[num].add("Nenhum")
            self.view.robot_bluetooths[num].value(0)

            current_item = 1
            for item in self.robot_bluetooth_keys:
                # Add the key of the dictionary to the drop-down
                self.view.robot_bluetooths[num].add(item)

                # If key is the same as the key in the robot
                if current_robot['bluetooth_mac_address'] == item:
                    # Set the value has they active item in the choice menu
                    self.view.robot_bluetooths[num].value(current_item)

                # Increments the value of item
                current_item += 1

            # This integer, again, is for the value of item in the
            # Drop-down choice box
            current_item = 0
            for item in self.robot_roles_keys:
                # Add the key of the dictionary to the drop-down...
                # again
                self.view.robot_roles[num].add(item)
                if current_robot['role'] == item:
                    # Same as above
                    self.view.robot_roles[num].value(current_item)
                # Well...
                current_item += 1

            # The value for the check button
            self.view.robot_radio_button[num].value(current_robot['active'])
            # Unique id for the check-Box
            self.view.robot_radio_button[num].id = num


    def send_vision_operation(self, operation):
        """Sends a service request to vision node
            :param operation : uint8

            :return: returns nothing
        """
        wait_for_service('vision_command')
        try:
            self.vision_proxy(operation)
        except ServiceException as exc:
            print("Service did not process request: " + str(exc))

    def top_menu_choice(self, ptr):
        if ptr.value() < 4:
            self.send_vision_operation(ptr.value())

        if ptr.value() < 9:
            if ptr.value() == 7: # Connection controller
                self.wait_window_close(self.connection_controller)
            if ptr.value() == 8: # Bluetooth Controller
                self.wait_window_close(self.bluetooth_controller)

        elif ptr.value() < 16:
            if ptr.value() == 13:
                self.wait_window_close(self.debug_controller)
            elif ptr.value() == 14:
                self.wait_window_close(self.connection_controller)

    def wait_window_close(self, window_controller):
        window_controller.show()
        while window_controller.view.root.visible():
            fl.Fl.wait()

    def action_input_choice(self, ptr):
        self.game_opt[self.assigned_robot_indexes[ptr.id]] = ptr.value()

    def radio_choice(self, ptr):
        self.trainer.set_robot_active(ptr.id, ptr.value())
        self.robot_params[self.faster_hash[ptr.id]]['active'] = ptr.value()

    def role_choice(self, ptr):
        self.pub.set_robot_role(ptr.id, ptr.value())
        self.pub.publish()
        self.robot_params[self.faster_hash[ptr.id]]['role'] = self.robot_roles_keys[ptr.value()]

    def bluetooth_choice(self, ptr):
        #this verification allow to set a default value for the bluetooth_name of the robot, so we can edit or delete bluetooth entries
        if ptr.value():
            self.robot_params[self.faster_hash[ptr.id]]['bluetooth_mac_address'] = self.robot_bluetooth_keys[ptr.value()]
            self.trainer.set_robot_bluetooth(ptr.id)
        else:
            self.robot_params[self.faster_hash[ptr.id]]['bluetooth_mac_address'] = "Nenhum"
            self.trainer.set_robot_active(ptr.id, False)

    def action_button_clicked(self, ptr):
        if self.view.play_button.playing:
            self.pub.set_game_state(0)  # Sets the game state to stopped
            self.pub.publish()

            for button in self.view.action_buttons:
                button.activate()
            ptr.color(fl.FL_DARK_GREEN)
            ptr.label("Jogar")
            self.view.play_button.playing = False
        else:
            if ptr.id == 4:
                self.pub.set_game_state(1)  # Sets the game state to normal play
                print("Jogar regular")
            elif ptr.id == 0:
                self.pub.set_game_state(2)
                print("Free Ball")
            elif ptr.id == 1:
                self.pub.set_game_state(3)
                print("Penalty")
            elif ptr.id == 2:
                self.pub.set_game_state(4)
                print("Meta")
            else:
                print("que")

            self.pub.publish()

            for button in self.view.action_buttons:
                button.deactivate()

            self.view.play_button.color(fl.FL_RED)
            self.view.play_button.label("Parar")
            self.view.play_button.playing = not self.view.play_button.playing

    #this function change the team color in the game_opt to the selected one
    #ptr is the pointer of the widget the callback belongs
    def on_color_change(self,ptr):
        ''':params ptr:pointer'''
        self.game_opt["time"] = ptr.value()

    #this function changes the team side in game_opt to the selected one
    #ptr is the pointer of the widget the callback belongs
    def on_side_change(self,ptr):
        ''':params ptr:pointer'''
        self.game_opt["side"] = ptr.value()

    def register_mac_service(self):
        """Creates a proxy for communicating with service
            :return: returns nothing
        """
        wait_for_service('vision_command')
        self.vision_proxy = ServiceProxy('vision_command', vision_command)
