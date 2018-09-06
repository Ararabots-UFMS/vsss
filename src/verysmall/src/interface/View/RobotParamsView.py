#!/usr/bin/python
# -*- coding: utf-8 -*-

import fltk as fl
import ctypes as ctype
import sys
import os

from time import sleep


def theCancelButtonCallback(ptr):
    os._exit(-1)


class RobotParamsView:
    """docstring for window_manager"""

    def __init__(self):
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()

        self.root = fl.Fl_Double_Window(self.proportion_width(25), self.proportion_height(40),
                                        self.proportion_width(50), self.proportion_height(15))

        self.width = self.root.w()
        self.height = self.root.h()

        self.check_simulation = None
        self.title_visao = None
        self.title_console = None
        self.check_robots = None
        self.top_menu = None
        self.line = None
        self.title = None

        self.root.label("Parâmetros do robô")
        self.create_main_field("Jogador 1")
        self.create_fields()
        self.create_buttons()

        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        self.root.show()
        self.root.end()

    def create_main_field(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(25),
                               self.proportion_width(20), self.proportion_height(20), text)

        self.title.color(fl.FL_RED)
        self.apply_bold_and_color(self.title, 0, True)
        self.title.align(fl.FL_ALIGN_RIGHT + fl.FL_ALIGN_INSIDE)
        self.title.show()

    def create_fields(self):

        self.tag_field = fl.Fl_Choice(self.proportion_width(28), self.proportion_height(25),
                                      self.proportion_width(7), self.proportion_height(20), "Tag:")

        self.bluetooth_field = fl.Fl_Choice(self.proportion_width(38), self.proportion_height(25),
                                            self.proportion_width(27), self.proportion_height(20), "Bluetooth:")

        self.body_field = fl.Fl_Choice(self.proportion_width(68), self.proportion_height(25),
                                       self.proportion_width(27), self.proportion_height(20), "Carcaça:")

        self.apply_input_label_style([self.tag_field, self.bluetooth_field, self.body_field])

    def create_buttons(self):
        self.cancel_button = fl.Fl_Button(self.proportion_width(63),
                                          self.proportion_height(70),
                                       self.proportion_width(15),
                                       self.proportion_height(20), "Cancel")

        self.ok_button = fl.Fl_Button(self.proportion_width(80),
                                      self.proportion_height(70),
                                   self.proportion_width(15),
                                   self.proportion_height(20), "OK")

        self.apply_bold_and_color(self.ok_button, fl.FL_DARK_GREEN, True)
        self.apply_bold_and_color(self.cancel_button, fl.FL_RED, True)

    def apply_input_label_style(self, array_of_widgets = None):
        for label in array_of_widgets:
            label.align(fl.FL_ALIGN_LEFT_TOP)
            label.labelcolor(fl.FL_WHITE)

    def apply_bold_and_color(self, label, color=0, flat_box = False):
        label.labelcolor(fl.FL_WHITE)
        label.labelfont(fl.FL_HELVETICA_BOLD)
        if color:
            label.color(color)

        if flat_box:
            label.box(fl.FL_FLAT_BOX)

    def proportion_height(self, proportion):
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        return int(self.width * proportion / 100)

    def end(self, hidden=False):
        # Show main window
        self.root.clear_visible_focus()
        self.root.end()
        if hidden:
            self.root.hide()
        else:
            self.root.show()
            fl.Fl.run()


if __name__ == '__main__':
    robotp = RobotParamsView()
    robotp.end()
    #fl.Fl.run()
