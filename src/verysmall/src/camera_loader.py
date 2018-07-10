#!/usr/bin/python
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



        else:
            self.camera_dev = search_index

    def camera_check(self):
        # List capture devices options
        devices = sb.check_output("ls", cwd="/sys/class/video4linux/").split("\n")

        # For each device in the dev dir
        for device_dir in devices:
            if len(device_dir):
                name = sb.check_output("cat name", cwd="/sys/class/video4linux/"+device_dir)
                input_folder = sb.check_output("ls", cwd="/sys/class/video4linux/"+device_dir+"device/input/").split("\n")
                product = sb.check_output("cat product", cwd="/sys/class/video4linux/"+device_dir+"device/input/"+input_folder+"/id/")
                vendor = sb.check_output("cat vendor", cwd="/sys/class/video4linux/" +device_dir+"device/input/"+input_folder+"/id/")
                self.camera_list[product] = {"name": name, "vendor": vendor}
                if product == self.camera_id:
                    return int(device_dir.strip("video"))

        return -1
