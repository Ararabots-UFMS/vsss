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

        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                        self.proportion_width(50), self.proportion_height(15))

        self.width = self.root.w()
        self.height = self.root.h()

        self.check_simulation = None
        self.title_visao = None
        self.title_console = None
        self.check_robots = None

        self.root.label("Parâmetros do rôbo")
        self.create_main_field("Jogador 1")
        self.create_fields()
        #self.create_title_visao("Visão")
        #self.create_title_console("Console")
        #self.create_check_robots()

        #self.create_check_simulation()
        #self.create_check_robot_vector()
        self.top_menu = None
        self.line = None
        self.title = None
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        self.root.show()
        self.root.end()

    def create_main_field(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(25),
                               self.proportion_width(20), self.proportion_height(20), text)
        self.title.labelcolor(fl.FL_WHITE)
        #self.title.labelsize(23)
        self.title.box(fl.FL_FLAT_BOX)
        self.title.color(fl.FL_RED)
        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_fields(self):

        self.tag_field = fl.Fl_Input(self.proportion_width(28), self.proportion_height(25),
                                    self.proportion_width(5), self.proportion_height(20), "Tag:")
        self.tag_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.tag_field.labelcolor(fl.FL_WHITE)

        self.bluetooth_field = fl.Fl_Input(self.proportion_width(36), self.proportion_height(25),
                                    self.proportion_width(25), self.proportion_height(20), "Bluetooth:")
        self.bluetooth_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.bluetooth_field.labelcolor(fl.FL_WHITE)

        self.body_field = fl.Fl_Input(self.proportion_width(64), self.proportion_height(25),
                                    self.proportion_width(25), self.proportion_height(20), "Carcaça:")
        self.body_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.body_field.labelcolor(fl.FL_WHITE)

    def create_title_visao(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(10),
                               self.proportion_width(35), self.proportion_height(4), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.box(fl.FL_NO_BOX)
        # self.title.color(fl.FL_RED)
        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_check_simulation(self):
        self.check_simulation = None
        self.check_simulation = fl.Fl_Check_Button(self.proportion_width(11),
                                                   self.proportion_height(15),
                                                   self.proportion_width(4), self.proportion_height(4),
                                                   "Simulação de previsão de movimento"
                                                   )
        self.check_simulation.labelcolor(fl.FL_WHITE)
        self.check_simulation.clear_visible_focus()

    def create_check_robot_vector(self):
        self.check_robot_vector = None
        self.check_robot_vector = fl.Fl_Check_Button(self.proportion_width(17),
                                                     self.proportion_height(20),
                                                     self.proportion_width(4), self.proportion_height(4),
                                                     "Vetores dos robôs"
                                                     )
        self.check_robot_vector.labelcolor(fl.FL_WHITE)
        self.check_robot_vector.clear_visible_focus()

    def create_title_console(self, text):
        self.title_console = fl.Fl_Box(self.proportion_width(5), self.proportion_height(26),
                                       self.proportion_width(35), self.proportion_height(4), text)
        self.title_console.labelcolor(fl.FL_WHITE)
        self.title_console.labelsize(23)
        self.title_console.box(fl.FL_FLAT_BOX)
        self.title_console.align(fl.FL_ALIGN_CENTER)
        self.title_console.show()

    def create_check_robots(self):
        n = 6
        self.check_robots = [None] * n
        temp_names = ["robo 1", "robo 2", "robo 3", "robo 4", "robo 5", "ball"]
        print temp_names
        for num in range(0, n / 2):
            self.check_robots[num] = fl.Fl_Check_Button(self.proportion_width(10) + self.proportion_width(5) * num * 2,
                                                        self.proportion_height(32),
                                                        self.proportion_width(4), self.proportion_height(4),
                                                        temp_names[num]
                                                        )
            self.check_robots[num].labelcolor(fl.FL_WHITE)
            self.check_robots[num].clear_visible_focus()

        for num in range(n / 2, n):
            self.check_robots[num] = fl.Fl_Check_Button(
                self.proportion_width(10) + self.proportion_width(5) * (num - n / 2) * 2,
                self.proportion_height(37), self.proportion_width(4), self.proportion_height(4),
                temp_names[num])

            self.check_robots[num].labelcolor(fl.FL_WHITE)
            self.check_robots[num].clear_visible_focus()
            self.check_robots[num].value(1)

    def create_scroll(self):
        self.scroll = fl.Fl_Scrollbar(self.proportion_width(5), self.proportion_height(26),
                                      self.proportion_width(35), self.proportion_height(20))

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

    fl.Fl.run()
