#!/usr/bin/python
# -*- coding: latin-1 -*-


class RobotParamsController:
    """This class controls the subwindow of Robot Parameters"""
    def __init__(self, _robot_params, _robot_bluetooth, _robot_roles):
        """
        :param _robot_params: Dict
        :param _robot_bluetooth: Dict
        :param _robot_roles: Dict
        :return: nothing
        """
        # Save the parameters for future use
        self.robot_params = _robot_params
        self.robot_bluetooth = _robot_bluetooth
        self.robot_roles = _robot_roles

        # Fast access array to use a dict as an simple array
        self.faster_hash = ['robot_' + str(x) for x in range(1, 6)]

        # Saves current robot owner of bluetooth
        self.bluetooth_is_owned_by = dict.fromkeys(self.robot_bluetooth.keys())
