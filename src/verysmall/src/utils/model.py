from utils.json_handler import JsonHandler

class Model():
    """The model class for loading and saving json files"""


    def __init__(self):
        self.json_handler = JsonHandler()
        self.robot_params = self.json_handler.read("parameters/robots.json", escape=True)
        self.robot_pid = self.json_handler.read("parameters/robots_pid.json", escape=True)
        self.robot_bluetooth = self.json_handler.read("parameters/bluetooth.json", escape=True)
        self.robot_roles = self.json_handler.read("parameters/roles.json", escape=True)
        self.game_opt = self.json_handler.read("parameters/game.json", escape=True)

    def save_params(self):
        self.json_handler.write(self.robot_params, "parameters/robots.json")
        self.json_handler.write(self.game_opt, "parameters/game.json")
        self.json_handler.write(self.robot_bluetooth, "parameters/bluetooth.json")
