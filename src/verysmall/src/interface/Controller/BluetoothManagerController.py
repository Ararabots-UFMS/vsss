#!/usr/bin/python
# -*- coding: latin-1 -*-

from ..View.BluetoothManagerView import BluetoothManagerView
import fltk as fl
from utils.json_handler import JsonHandler

class BluetoothManagerController():
    def __init__(self):
        self.file = "parameters/bluetooth.json"
        self.json_handler = JsonHandler()
        self.bluetooths_dict = self.json_handler.read("parameters/bluetooth.json")
        self.view = BluetoothManagerView()
        self.view.root.callback(self.on_close_callback)
        buffer = []
        for key in self.bluetooths_dict.keys():
            buffer.append([str(key), str(self.bluetooths_dict[key])])
        for b in buffer:
            self.view.create_bluetooth_entry(b[0],b[1])
        self.view.end()

    def on_close_callback(self,ptr):
        print "maoe"
        self.bluetooths_dict = {}
        for b in self.view.bluetooths:
            key = str(b[0].label())
            value = str(b[1].label())
            self.bluetooths_dict[key] = value
        self.json_handler.write(self.bluetooths_dict,"parameters/bluetooth.json")
        self.view.root.hide()
