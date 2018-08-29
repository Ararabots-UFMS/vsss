#!/usr/bin/python
# -*- coding: utf-8 -*-

import fltk as fl
import ctypes as ctype
import sys
import os

from time import sleep


def theCancelButtonCallback(ptr):
    os._exit(-1)


class ConnectionView:
    """docstring for window_manager"""

    def __init__(self):
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()
        
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                 self.proportion_width(45), self.proportion_height(55))

        self.width = self.root.w()
        self.height = self.root.h()

        self.root.label("Conexao")
        self.top_menu = None
        self.line = None
        self.title = None
        self.node_title = None
        self.bluetooth_title = None
        self.carcaca_title = None
        self.choice_jogador = []
        self.check_robots = None
        self.ip_field = None
        self.update_button = None
        self.joystick_title = None
        self.create_main_title("Node mestre")
        #self.create_bluetooth_title("Bluetooth")
        #self.create_carcaca_title("Carcaça")
        #self.create_joystick_title("Joystick")
        #self.create_node_title("Node mestre")
        # self.create_player_number()
        #self.create_choice_bluetooth()
        #self.create_choice_carcaca()
        #self.create_check_robots()
        self.create_ip_field()
        self.create_update_button("Atualizar")
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        self.root.end()

        #while fl.Fl.wait() > 0:
        #    if fl.Fl.get_key(ord("q")):#ord pega o numero do caractere dentro da funcd
        #        fl._exit()#sair da janela

    def create_main_title(self, text):
        self.title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(0),
                               self.proportion_width(35), self.proportion_height(10), text)
        self.title.labelcolor(fl.FL_WHITE)
        self.title.labelsize(23)
        self.title.box(fl.FL_FLAT_BOX)
        # self.title.color(fl.FL_RED)
        self.title.align(fl.FL_ALIGN_CENTER)
        self.title.show()

    def create_node_title(self, text):
        self.node_title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(33),
                               self.proportion_width(35), self.proportion_height(10), text)
        self.node_title.labelcolor(fl.FL_WHITE)
        self.node_title.labelsize(23)
        self.node_title.box(fl.FL_FLAT_BOX)
        # self.title.color(fl.FL_RED)
        self.node_title.align(fl.FL_ALIGN_CENTER)
        self.node_title.show()

    def create_bluetooth_title(self,text):
        self.bluetooth_title = fl.Fl_Box(self.proportion_width(5), self.proportion_height(8),
                               self.proportion_width(15), self.proportion_height(7), text)
        self.bluetooth_title.labelcolor(fl.FL_WHITE)
        self.bluetooth_title.labelsize(15)
        self.bluetooth_title.box(fl.FL_FLAT_BOX)
        # self.bluetooth_title.color(fl.FL_RED)
        self.bluetooth_title.show()

    def create_carcaca_title(self,text):
        self.carcaca_title = fl.Fl_Box(self.proportion_width(25), self.proportion_height(8),
                               self.proportion_width(15), self.proportion_height(7), text)
        self.carcaca_title.labelcolor(fl.FL_WHITE)
        self.carcaca_title.labelsize(15)
        self.carcaca_title.box(fl.FL_FLAT_BOX)
        # self.carcaca_title.color(fl.FL_RED)
        self.carcaca_title.show()

    def create_joystick_title(self,text):
        self.joystick_title = fl.Fl_Box(self.proportion_width(35), self.proportion_height(8),
                               self.proportion_width(15), self.proportion_height(7), text)
        self.joystick_title.labelcolor(fl.FL_WHITE)
        self.joystick_title.labelsize(15)
        self.joystick_title.box(fl.FL_FLAT_BOX)
        # self.carcaca_title.color(fl.FL_RED)
        self.joystick_title.show()

    def create_player_number(self):
        self.carcaca_title = fl.Fl_Box(self.proportion_width(3), self.proportion_height(15),
                               self.proportion_width(5), self.proportion_height(10), "1")
        self.carcaca_title.labelcolor(fl.FL_WHITE)
        self.carcaca_title.labelsize(15)
        self.carcaca_title.show()

        self.carcaca_title = fl.Fl_Box(self.proportion_width(3), self.proportion_height(20),
                               self.proportion_width(5), self.proportion_height(10), "2")
        self.carcaca_title.labelcolor(fl.FL_WHITE)
        self.carcaca_title.labelsize(15)
        self.carcaca_title.show()

        self.carcaca_title = fl.Fl_Box(self.proportion_width(3), self.proportion_height(25),
                               self.proportion_width(5), self.proportion_height(10), "3")
        self.carcaca_title.labelcolor(fl.FL_WHITE)
        self.carcaca_title.labelsize(15)
        self.carcaca_title.show()

    def create_choice_bluetooth(self):
        self.choice_bluetooth = [None, None, None]
        temp_names = ["1", "2", "3"]
        for num in range(3):
            self.choice_bluetooth[num] = fl.Fl_Choice(self.proportion_width(5), self.proportion_height(15) +
                                         self.proportion_height(3)*num*2, self.proportion_width(15),
                                         self.proportion_height(4), temp_names[num])
            self.choice_bluetooth[num].labelcolor(fl.FL_WHITE)
            self.choice_bluetooth[num].color(fl.FL_RED)
            self.choice_bluetooth[num].show()

    def create_choice_carcaca(self):
        self.choice_carcaca = [None, None, None]
        temp_names = ["1", "2", "3"]
        for num in range(3):
            self.choice_carcaca[num] = fl.Fl_Choice(self.proportion_width(25), self.proportion_height(15) +
                                         self.proportion_height(3)*num*2, self.proportion_width(15),
                                         self.proportion_height(4), temp_names[num])
            self.choice_carcaca[num].labelcolor(fl.FL_WHITE)
            self.choice_carcaca[num].color(fl.FL_RED)
            self.choice_carcaca[num].show()

    def create_check_robots(self):
        self.check_robots = [None, None, None]
        temp_names = ["1", "2", "3"]
        for num in range(3):
            self.check_robots[num] = fl.Fl_Check_Button(self.proportion_width(12)*3 + self.proportion_width(5),
                               self.proportion_height(15) + self.proportion_height(3) * num * 2,
                               self.proportion_width(4), self.proportion_height(4)
                               )
            self.check_robots[num].clear_visible_focus()

    def create_ip_field(self):
        self.ip_field = fl.Fl_Input(self.proportion_width(10), self.proportion_height(15),
                               self.proportion_width(20), self.proportion_height(5), "IP Nó Mestre:")
        self.ip_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.ip_field.labelcolor(fl.FL_WHITE)
        self.ip_field.show()

        self.self_ip_field = fl.Fl_Choice(self.proportion_width(10), self.proportion_height(25),
                               self.proportion_width(20), self.proportion_height(5), "Próprio ROS IP:")
        self.self_ip_field.align(fl.FL_ALIGN_LEFT_TOP)
        self.self_ip_field.labelcolor(fl.FL_WHITE)
        self.self_ip_field.show()

    def create_update_button(self,text):
        self.update_button = fl.Fl_Button(self.proportion_width(15), self.proportion_height(30),
                               self.proportion_width(6), self.proportion_height(5), text)
        self.update_button.color(fl.FL_RED)
        self.update_button.labelcolor(fl.FL_WHITE)
        self.update_button.labelfont(fl.FL_BOLD)

    def proportion_height(self, proportion):
        return int(self.height * proportion/100)

    def proportion_width(self, proportion):
        return int(self.width * proportion/100)


if __name__ == '__main__':

    window_manager = ConnectionView()
    window_manager.root.show()
    fl.Fl.run()