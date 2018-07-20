from ..View.MainWindowView import MainWindowView
import fltk as fl


class MainWindowController():
    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles, _game_opt):
        # Lets create the view of our controller shall we
        self.view = MainWindowView()

        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles
        self.game_opt = _game_opt

        # Since our primary keys are the keys of the dict it self
        # and since our DataBase is simple, it can be stored as simple strings
        self.robot_roles_keys = self.robot_roles.keys()
        self.robot_bluetooth_keys = self.robot_bluetooth.keys()

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_'+str(x) for x in range(1, 6)]
        self.assigned_robot_text = ["Jogador "+str(x) for x in range(1, 6)]
        self.assigned_robot_indexes = ['penalty_player', 'freeball_player', 'meta_player']

        # For each Robot, this loop covers all the inputs
        for num in range(self.view.n_robots):
            # Access the robot params dict using a int
            # and stores its reference in a temporary variable
            current_robot = self.robot_params[self.faster_hash[num]]

            # This integer is for the value of item in the
            # Drop-down choice box
            current_item = 0
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

            # Multiple callbacks, each for one type of input
            # but since whe have ids for each robot input
            # we can parse through each using its on dictionary
            self.view.robot_bluetooths[num].callback(self.bluetooth_choice)
            self.view.robot_roles[num].callback(self.role_choice)
            self.view.robot_radio_button[num].callback(self.radio_choice)

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

        self.view.play_button.callback(self.action_button_clicked)

        self.view.end()

    def action_input_choice(self, ptr):
        self.game_opt[self.assigned_robot_indexes[ptr.id]] = ptr.value()

    def radio_choice(self, ptr):
        self.robot_params[self.faster_hash[ptr.id]]['active'] = ptr.value()

    def role_choice(self, ptr):
        self.robot_params[self.faster_hash[ptr.id]]['role'] = self.robot_roles_keys[ptr.value()]

    def bluetooth_choice(self, ptr):
        self.robot_params[self.faster_hash[ptr.id]]['bluetooth_mac_address'] = self.robot_bluetooth_keys[ptr.value()]

    def action_button_clicked(self, ptr):
        if self.view.play_button.playing:
            for button in self.view.action_buttons:
                button.activate()
            ptr.color(fl.FL_DARK_GREEN)
            ptr.label("Jogar")
            self.view.play_button.playing = False
        else:
            if ptr.id == 4:
                print("Jogar regular")
            elif ptr.id == 0:
                print("Free Ball")
            elif ptr.id == 1:
                print("Penalty")
            elif ptr.id == 2:
                print("Meta")
            else:
                print("que")

            for button in self.view.action_buttons:
                button.deactivate()

            self.view.play_button.color(fl.FL_RED)
            self.view.play_button.label("Parar")
            self.view.play_button.playing = not self.view.play_button.playing
