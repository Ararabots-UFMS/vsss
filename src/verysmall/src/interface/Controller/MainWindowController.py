from ..View.MainWindowView import MainWindowView
from BluetoothManagerController import BluetoothManagerController
from ConnectionController import ConnectionController
from DebugController import DebugController
from RobotParamsController import RobotParamsController
import fltk as fl
import sys
import os

# old_path = sys.path[0]
# sys.path[0] = os.environ['ROS_ARARA_ROOT'] + "src/"
# from vision.vision_node import VisionOperations
# sys.path[0] = old_path


class MainWindowController:
    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _game_opt, _debug_params, _robot_bodies, _coach
                 , _game_topic_publisher):

        # The controllers are created but not show
        self.bluetooth_controller = BluetoothManagerController(_robot_bluetooth, hidden=True)
        self.connection_controller = ConnectionController(_game_opt)
        self.debug_controller = DebugController(_debug_params, hidden=True)
        self.robot_params_controller = RobotParamsController(_robot_params, _robot_bluetooth,
                                                             _robot_roles, _robot_bodies)

        # Lets create the view of our controller shall we
        self.view = MainWindowView()

        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.game_opt = _game_opt

        # The coach object class control the active and the activities of robots
        self.coach = _coach

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
        self.pub = _game_topic_publisher

        #replaced by the function set_robots_bluetooth
        self.set_robots_params()

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
            self.view.robot_params[num].callback(self.parameters_button)
            self.view.robot_roles[num].callback(self.role_choice)
            self.view.robot_radio_button[num].callback(self.radio_choice)
            
        self.view.top_menu.callback(self.top_menu_choice)
        self.view.play_button.callback(self.action_button_clicked)

        self.view.team_color.callback(self.on_color_change)
        self.view.team_side.callback(self.on_side_change)

        self.view.end()

    def set_robots_params(self):
        # For each Robot, this loop covers all the inputs
        for num in xrange(self.view.n_robots):
            # Access the robot params dict using a int
            # and stores its reference in a temporary variable
            current_robot = self.robot_params[self.faster_hash[num]]

            # This integer is for the value of item in the
            # Drop-down choice box
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

    def parameters_button(self, ptr):
        old = self.robot_params[self.faster_hash[ptr.id]].copy()

        self.robot_params_controller.show(ptr.id)
        while self.robot_params_controller.view.root.visible():
            fl.Fl.wait()

        the_same = self.robot_params[self.faster_hash[ptr.id]] == old

        if (not the_same) and self.robot_params[self.faster_hash[ptr.id]]['active']:
            self.coach.set_robot_parameters(ptr.id)

    def top_menu_choice(self, ptr):
        if ptr.value() < 4:
            self.pub.send_vision_operation(ptr.value())

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
        self.coach.set_robot_active(ptr.id, ptr.value())
        self.robot_params[self.faster_hash[ptr.id]]['active'] = ptr.value()

    def role_choice(self, ptr):
        self.coach.change_robot_role(ptr.id, ptr.value())
        self.robot_params[self.faster_hash[ptr.id]]['role'] = self.robot_roles_keys[ptr.value()]

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

    def on_color_change(self, ptr):
        """
        This function change the team color in the game_opt to the selected one
        :param ptr: pointer of the widget
        :return: nothing
        """
        self.pub.send_vision_operation(ptr.value()+4)  # Defined in VisionOperations - Vision Node file
        self.game_opt["time"] = ptr.value()

    def on_side_change(self, ptr):
        """
        This function changes the team side in game_opt to the selected one
        :param ptr: pointer of the widget
        :return: nothing
        """
        self.pub.set_side_of_the_field(ptr.value())
        self.game_opt["side"] = ptr.value()
        self.pub.publish()
