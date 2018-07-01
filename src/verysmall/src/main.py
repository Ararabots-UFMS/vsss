from utils.json_handler import JsonHandler
from interface.gui import window_manager
"""
Instantiates all the windows, robots, topics and services
"""


class Model():
    """The model class for loading and saving json files"""

    def __init__(self):
        self.json_handler = JsonHandler()
        self.robot_params = self.json_handler.read("parameters/robots.json")
        self.robot_pid = self.json_handler.read("parameters/robots_pid.json")
        self.robot_bluetooth = self.json_handler.read("parameters/bluetooth.json")


if __name__ == '__main__':
    # Load the database
    model = Model()

    # Create the GUI


    print(model.robot_params.keys())
    print(model.robot_pid.keys())
    print(model.robot_bluetooth.keys())