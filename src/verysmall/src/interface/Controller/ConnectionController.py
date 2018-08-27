#!/usr/bin/pythonderp
# -*- coding: latin-1 -*-
from ..View.ConnectionView import ConnectionView
from os import environ
from fltk import fl_message, FL_WHITE, Fl

class ConnectionController:
    def __init__(self, _robot_params, _game_opt):
        self.robot_params = _robot_params
        self.view = ConnectionView()
        self.default_ros_uri = "http://localhost:11311"
        self.game_opt = _game_opt
        self.file_master_uri = self.game_opt["ROS_MASTER_URI"]
        self.view.ip_field.value(self.file_master_uri)

        self.view.update_button.callback(self.change_ros_master)

    def show(self):
        self.view.root.show()

    def change_ros_master(self, ptr):
        self.file_master_uri = self.view.ip_field.value()
        line = "export ROS_MASTER_URI="+self.file_master_uri

        with open(environ['ROS_ARARA_ROOT'] + "src/parameters/rosmaster.bash", "w+") as bash_file:
            self.game_opt["ROS_MASTER_URI"] = self.file_master_uri
            Fl.background(200, 200, 200)
            fl_message("Por favor, reinicie o bash para concluir as alterações")
            bash_file.write(line)
            bash_file.close()
