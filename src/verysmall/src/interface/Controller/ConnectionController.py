#!/usr/bin/pythonderp
# -*- coding: latin-1 -*-
from ..View.ConnectionView import ConnectionView
from os import environ
from sys import path
from fltk import fl_message, FL_WHITE, Fl
path[0] = environ['ROS_ARARA_ROOT'] + "src/"
from utils.enum_interfaces import all_interfaces, format_ip

class ConnectionController:
    """
        This class is responsible for creating and updating the connection view

        :return: returns nothing
    """

    def __init__(self, _robot_params, _game_opt):
        self.robot_params = _robot_params
        self.view = ConnectionView()
        self.default_ros_uri = "http://localhost:11311"
        self.game_opt = _game_opt
        self.file_master_uri = self.game_opt["ROS_MASTER_URI"]
        self.view.ip_field.value(self.file_master_uri)

        self.view.update_button.callback(self.change_ros_master)
        self.view.self_ip_field.callback(self.ip_choice)

        item = -1
        count = 0
        self.ip_list = all_interfaces()

        for ip in self.ip_list:
            self.view.self_ip_field.add(ip[0]+" - "+format_ip(ip[1]))
            if ip[0] == self.game_opt['ROS_IP'][0] and format_ip(ip[1]) == self.game_opt['ROS_IP'][1]:
                item = count
            count += 1

        if item == -1:
            item = count
            self.view.self_ip_field.add(self.game_opt['ROS_IP'][0] + " - " + self.game_opt['ROS_IP'][1])
            self.ip_list.append(self.game_opt['ROS_IP'])

        self.view.self_ip_field.value(item)
        self.ip = self.ip_list[item]

    def show(self):
        """
            Displays the connection view
            :return: returns nothing
        """
        self.view.root.show()

    def ip_choice(self, ptr):
        """
            This function sets the ip variable for saving
            :param ptr: pointer for the widget, uses value() attribute
            :return: returns nothing
        """
        self.ip = self.ip_list[ptr.value()]

    def change_ros_master(self, ptr):
        """
            This function is for updating the bash and json

            :param ptr: note used but required for callback
            :return: returns nothing
        """

        self.file_master_uri = self.view.ip_field.value()
        formatted_ip_string = format_ip(self.ip[1])
        self.game_opt["ROS_MASTER_URI"] = self.file_master_uri
        self.game_opt["ROS_IP"] = [self.ip[0], formatted_ip_string]

        line = "export ROS_MASTER_URI="+self.file_master_uri + "\n" +\
               "export ROS_HOSTNAME="+formatted_ip_string + "\n" +\
               "export ROS_IP="+formatted_ip_string + "\n"

        with open(environ['ROS_ARARA_ROOT'] + "src/parameters/rosmaster.bash", "w+") as bash_file:
            bash_file.write(line)
            bash_file.close()
            Fl.background(200, 200, 200)
            fl_message("Por favor, reinicie o bash para concluir as alterações")
