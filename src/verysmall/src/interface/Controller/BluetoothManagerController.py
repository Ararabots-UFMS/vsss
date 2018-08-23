#!/usr/bin/python
# -*- coding: latin-1 -*-

from ..View.BluetoothManagerView import BluetoothManagerView
import fltk as fl
from utils.json_handler import JsonHandler

class BluetoothManagerController():
    def __init__(self, _robot_bluetooth, hidden=False):
        self.bluetooths_dict = _robot_bluetooth#self.json_handler.read("parameters/bluetooth.json")
        self.view = BluetoothManagerView()
        self.view.root.callback(self.on_close_callback)
        buffer = []
        for key in self.bluetooths_dict.keys():
            buffer.append([str(key), str(self.bluetooths_dict[key])])
        for b in buffer:
            self.view.create_bluetooth_entry(b[0],b[1])
        self.view.end(hidden)

    def on_close_callback(self,ptr):
        bluetooths = {}
        for b in self.view.bluetooths:
            name, address = b[0].label(), b[1].label()
            bluetooths = [name] = address
        jh = JsonHandler()
        jh.write(bt_dict,"parameters/bluetooth.json")
        self.view.root.hide()

    def show(self):
        self.view.root.show()



if __name__ == '__main__':
    window_manager = BluetoothManagerController()
