#!/usr/bin/python
# -*- coding: latin-1 -*-
import sys
import numpy as np
import fltk as fl
from ..virtualField import virtualField
import rospy
import time
from Queue import Queue
from collections import deque
from verysmall.msg import things_position


class canvas(fl.Fl_Double_Window):
    def __init__(self, x, y, w, h, l, image):
        fl.Fl_Double_Window.__init__(self, x, y, w, h, l)
        self.image = image

    def draw(self):
        fl.fl_draw_image(self.image.data, 0,0, self.w(), self.h(), 3, 0)

class MainWindowView:
    """Creates the visual for the main window and uses the MainWindowController to handle callbacks"""

    def __init__(self):

        # Init Variables
        self.title = None
        self.option_robots = [None, None, None, None, None]
        self.robot_bluetooths = [None, None, None, None, None]
        self.robot_roles = [None, None, None, None, None]
        self.team_color = None
        self.team_side = None
        self.robot_radio_button = [None, None, None, None, None]
        self.action_buttons = []
        self.play_button = None
        self.top_menu = None
        self.line = None
        self.image_container = None
        self.padding_x = 0
        self.padding_y = 0
        self.n_robots = 5

        # Get the usable screen proportions
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()

        # Forces double buffering
        fl.Fl.visual(fl.FL_DOUBLE | fl.FL_INDEX)

        # Create a new Buffered window
        self.root = fl.Fl_Window(self.proportion_width(2.5), self.proportion_height(5),
                                        self.proportion_width(95), self.proportion_height(90))
        #TODO: Diminuir campo virtual para abrir space para os numeros da tag
        #TODO: Criar dropdown para as tags
        self.virtualField = virtualField(self.proportion_width(50), self.proportion_height(70), is_rgb=True)
        self.virtualField.plot_arena(self.virtualField.raw_field)

        self.root.label("ARARABOTS MANAGEMENT SYSTEM")

        # Construct main window
        self.create_top_menu()
        self.create_left_menu()
        self.create_arena()
        self.create_toggle_color()
        self.create_toggle_side()

        # Queue of data from Topic Things position
        #self.data = Queue(maxsize=10)
        msg = things_position()
        self.data = deque(
        # Shapes the size of the Queue
        #self.data.put(
            [msg,msg])

        # Ros node for reading the buffer
        rospy.Subscriber('things_position', things_position, self.read, queue_size=1)
        rospy.init_node('virtual_field', anonymous=True)
        self.past_time = time.time()
        # Define colors
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)

    def read(self, data):
        # Inserts data in the Queue
        #if not self.data.full():
        self.data.append(data)

    def redraw_field(self):
        #if not self.data.empty():
        try:
            self.data_item = data_item = self.data.popleft()  # Get the data
            #rospy.logfatal(len(self.data))
            #self.data.task_done()  # Finishes the get process
            #self.now_time = time.time()
            #rospy.logfatal(self.now_time - self.past_time)

            self.virtualField.plot_ball(np.nan_to_num(np.array(data_item.ball_pos)))  # Plot the ball
            self.virtualField.plot_robots(np.nan_to_num(np.array(data_item.team_pos)).reshape((5, 2)),
                                           np.nan_to_num(np.array(data_item.team_orientation)),
                                           self.virtualField.colors["yellow"])

            self.virtualField.plot_robots(np.nan_to_num(data_item.enemies_pos).reshape((5, 2)),
                                           np.nan_to_num(data_item.enemies_orientation),
                                           self.virtualField.colors["blue"], is_away=True)
            self.arena.image = self.virtualField.field
            self.arena.redraw()
            #self.past_time = self.now_time
        #else:
        except IndexError:
            #print("vazia")
            rospy.loginfo("vazia")
            #self.arena.redraw()

        fl.Fl.repeat_timeout(self.RATE, self.redraw_field)

    def create_arena(self):
        """Creates a top window, double buffered one"""
        self.arena = canvas(self.proportion_width(40), self.proportion_height(15),
                            self.proportion_width(50), self.proportion_height(70), "D_Window",self.virtualField.field)
        self.arena.box(fl.FL_FLAT_BOX)
        self.arena.end()

    def create_top_menu(self):
        """Creates the buttons and inputs for players"""
        self.padding_y += self.proportion_height(5)
        self.top_menu = fl.Fl_Menu_Bar(0, 0, self.width, self.padding_y)
        self.top_menu.box(fl.FL_NO_BOX)
        self.top_menu.labelcolor(fl.FL_WHITE)
        self.top_menu.textcolor(fl.FL_WHITE)

        self.top_menu.add("Câmera", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Câmera/Calibração")
        self.top_menu.add("Câmera/Corte")
        self.top_menu.add("Câmera/Opções")

        self.top_menu.add("Jogadores", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Jogadores/Calibração")
        self.top_menu.add("Jogadores/Carcaças")
        self.top_menu.add("Jogadores/Bluetooth")

        self.top_menu.add("Configurações", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Configurações/Interface")
        self.top_menu.add("Configurações/Campo Virtual")
        self.top_menu.add("Configurações/Console")

        self.top_menu.add("Sobre", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Sobre/Hotkeys")
        self.top_menu.add("Sobre/Interface")
        self.top_menu.add("Sobre/Equipe")

        self.line = fl.Fl_Box(0, self.padding_y, self.width, 3)
        self.line.box(fl.FL_FLAT_BOX)
        self.line.color(fl.FL_RED)
        self.line.show()

    def create_left_menu(self):
        # initial padding for top
        self.padding_y += self.proportion_height(3)

        # ========== Titulo ============================
        self.title = fl.Fl_Box(self.proportion_width(10),
                               self.padding_y,
                               self.proportion_width(23),
                               self.proportion_height(13), "Ararabots")
        self.title.labelcolor(fl.FL_WHITE)  # color
        self.title.labelsize(26)  # Font Size
        self.title.labelfont(fl.FL_HELVETICA_BOLD)  # Bold type
        self.title.show()  # Show title

        # Padding for the upper labels
        self.padding_y += self.proportion_height(10)
        self.padding_x = self.proportion_width(10)
        # No loop unrolling this time
        for text in ["Papel", "Camisa"]:
            label = fl.Fl_Box(self.padding_x,
                              self.padding_y,
                              self.proportion_width(10),
                              self.proportion_height(5),
                              text)
            label.labelcolor(fl.FL_WHITE)
            label.labelfont(fl.FL_HELVETICA_BOLD)  # type Bold
            self.padding_x += self.proportion_width(11)

        # Variables for the input loops
        temp_names = ["Jogador 1: ", "Jogador 2: ", "Jogador 3: ", "Jogador 4: ", "Jogador 5: "]
        temp_x_padding = [self.proportion_width(10),
                          self.proportion_width(10) * 2 + self.proportion_width(1),
                          self.proportion_width(10) * 3 + self.proportion_width(2)
                          ]
        self.padding_y += self.proportion_height(5)

        for num in range(self.n_robots):
            # Defines choice input
            self.robot_roles[num] = fl.Fl_Choice(temp_x_padding[0],
                                                  self.padding_y, self.proportion_width(10),
                                                  self.proportion_height(4),
                                                  temp_names[num])

            # ID for using in callback with Robot roles
            self.robot_roles[num].id = num

            # Body Input styles
            self.robot_roles[num].color(fl.FL_RED)
            self.robot_roles[num].labelcolor(fl.FL_WHITE)
            self.robot_roles[num].box(fl.FL_NO_BOX)

            # Bluetooth inputs
            self.robot_bluetooths[num] = fl.Fl_Choice(
                temp_x_padding[1],
                self.padding_y,
                self.proportion_width(10),
                self.proportion_height(4),
                temp_names[num])

            # ID for using in callback with the bluetooth input
            self.robot_bluetooths[num].id = num

            # Bluetooth inputs styles
            self.robot_bluetooths[num].color(fl.FL_RED)
            self.robot_bluetooths[num].labeltype(fl.FL_NO_LABEL)
            self.robot_bluetooths[num].labelcolor(fl.FL_WHITE)

            # Input to define if robot is active or not
            self.robot_radio_button[num] = fl.Fl_Check_Button(temp_x_padding[2],
                               self.padding_y,
                               self.proportion_width(2),
                               self.proportion_height(4)
                               )
            # Padding for next line
            self.padding_y += self.proportion_height(3) * 2

        self.padding_y += self.proportion_height(2)

        temp_names = ["Freeball: ", "Penalti: ", "Tiro de meta: "]
        for num in range(3):
            self.option_robots[num] = fl.Fl_Choice(self.proportion_width(10),
                                                   self.padding_y,
                                                   self.proportion_width(10),
                                                   self.proportion_height(4),
                                                   temp_names[num])

            self.option_robots[num].id = num
            self.option_robots[num].down_box(fl.FL_FLAT_BOX)
            self.option_robots[num].labelcolor(fl.FL_WHITE)
            self.option_robots[num].color(fl.FL_RED)

            action_button = fl.Fl_Button(self.proportion_width(10) * 2 + self.proportion_width(1),
                                         self.padding_y,
                                         self.proportion_width(10),
                                         self.proportion_height(4),
                                         "Vai!")
            action_button.labelcolor(fl.FL_WHITE)
            action_button.color(fl.FL_DARK_GREEN)
            action_button.labelfont(fl.FL_BOLD)
            action_button.id = num
            self.action_buttons.append(action_button)

            self.padding_y += self.proportion_height(3) * 2

        self.padding_y += self.proportion_height(3)

        play_pause_button = fl.Fl_Button(self.proportion_width(10),
                                         self.padding_y + self.proportion_height(3) * 2,
                                         self.proportion_width(21),
                                         self.proportion_height(4),
                                         "Jogar"
                                         )
        play_pause_button.id = 4
        play_pause_button.playing = False
        play_pause_button.labelcolor(fl.FL_WHITE)
        play_pause_button.labelfont(fl.FL_BOLD)
        play_pause_button.box(fl.FL_FLAT_BOX)
        play_pause_button.color(fl.FL_DARK_GREEN)
        self.play_button = play_pause_button

    def create_toggle_color(self):
        #creates a dropdown menu to choose the color of the team
        
        temp_name = "Cor da Camisa"
        self.team_color = fl.Fl_Choice(self.proportion_width(10),
                                               self.padding_y,
                                               self.proportion_width(10),
                                               self.proportion_height(4),
                                               temp_name)

        self.team_color.id = 13
        self.team_color.down_box(fl.FL_FLAT_BOX)
        self.team_color.labelcolor(fl.FL_WHITE)
        self.team_color.color(fl.FL_RED)
        self.team_color.add("Azul")
        self.team_color.add("Amarelo")
        self.team_color.value(0)
       # self.team_color.align(fl.FL_ALIGN_LEFT)
        #self.padding_y += self.proportion_height(3) * 2
        #we skip this padding increment to force the next component to be with the same height

    def create_toggle_side(self):
     #creates a dropdown menu to choose the color of the team
        
        temp_name = "Lado da arena"
        self.team_side = fl.Fl_Choice(self.proportion_width(10)  + self.proportion_width(11),
                                               self.padding_y,
                                               self.proportion_width(10),
                                               self.proportion_height(4),
                                               temp_name)

        self.team_side.id = 13
        self.team_side.down_box(fl.FL_FLAT_BOX)
        self.team_side.labelcolor(fl.FL_WHITE)
        self.team_side.color(fl.FL_RED)
        self.team_side.add("Direito")
        self.team_side.add("Esquerdo")
        self.team_side.value(0)
        self.team_side.align(fl.FL_ALIGN_RIGHT)
        self.padding_y += self.proportion_height(3) * 2
        
    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

    def end(self):
        # Show main window
        self.root.clear_visible_focus()
        self.root.end()
        self.root.show()

        self.RATE = 0.013#0.04

        fl.Fl.add_timeout(self.RATE, self.redraw_field)

        fl.Fl.run()

if __name__ == '__main__':
    mainwindow = MainWindowView()
