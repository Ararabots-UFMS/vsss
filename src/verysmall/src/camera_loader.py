#!/usr/bin/python
from interface.Controller.CameraSelectController import CameraSelectController
import sys, os
import subprocess as sb


class CameraLoader:

    def __init__(self, _camera_id):
        # Product ID
        self.camera_id = _camera_id
        # List of possible devices
        self.camera_list = {}

        search_index = self.camera_check()

        if search_index == -1:
            self.controller = CameraSelectController(self.camera_id, self.camera_list, self.camera_check)
        else:
            self.camera_dev = search_index

    def camera_check(self):
        self.camera_list.clear()
        # List capture devices options
        devices = sb.check_output("ls", cwd="/sys/class/video4linux/").split("\n")

        # For each device in the dev dir
        for device_dir in devices:
            if len(device_dir):
                name = sb.check_output("cat /sys/class/video4linux/"+device_dir+"/name", shell=True).strip("\n")
                input_folder = sb.check_output("ls", cwd="/sys/class/video4linux/"+device_dir+"/device/input/").strip("\n")
                product = sb.check_output("cat /sys/class/video4linux/"+device_dir+"/device/input/"+input_folder+"/id/product", shell=True).strip("\n")
                #vendor = sb.check_output("cat /sys/class/video4linux/"+device_dir+"/device/input/"+input_folder+"/id/vendor", shell=True).strip("\n")
                device_number = device_dir.strip("video")
                self.camera_list[device_number] = {"name": name, "product": product}
                if product == self.camera_id:
                    return int(device_number)

        return -1


if __name__ == '__main__':
    CameraLoader('poopy')