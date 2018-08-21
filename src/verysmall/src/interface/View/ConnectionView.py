#!/usr/bin/python
# -*- coding: utf-8 -*-

import fltk as fl
import ctypes as ctype
import sys
import os


def theCancelButtonCallback(ptr):
    os._exit(-1)


class ConnectionView:
    """docstring for window_manager"""

    def __init__(self):
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                 self.proportion_width(45), self.proportion_height(55))

        self.top_menu = None
        self.line = None
        self.title = None
        self.bluetooth_title = None
        self.carcaca_title = None
        # self.create_button("olar")
        self.create_main_title("Jogadores")
        self.create_bluetooth_title("Bluetooth")
        self.create_carcaca_title("Carcaça")
        self.root.end()
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        self.root.show(len(sys.argv), sys.argv)

    def create_main_title(self, text):
        self.title = fl.Fl_Box(self.proportion_width(11), self.proportion_height(0),
                               self.proportion_width(23), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.show()

    def create_bluetooth_title(self,text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(10),
                               self.proportion_width(15), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(15)
        self.title.show()

    def create_carcaca_title(self,text):
        self.title = fl.Fl_Box(self.proportion_width(25), self.proportion_height(10),
                               self.proportion_width(15), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(15)
        self.title.show()


    def create_button(self,text):
         button = fl.Fl_Button(10, 10, 40, 20, text).align(fl.FL_ALIGN_CENTER)

    def proportion_height(self, proportion):
        return int(self.height * proportion/100)

    def proportion_width(self, proportion):
        return int(self.width * proportion/100)


if __name__ == '__main__':

    window_manager = ConnectionView()

    fl.Fl.run()
