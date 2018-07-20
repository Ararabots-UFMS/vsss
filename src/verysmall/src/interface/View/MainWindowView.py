#!/usr/bin/python
# -*- coding: latin-1 -*-
import sys
import fltk as fl
from ..virtual_field import Virtual_Field
import rospy
from Queue import Queue
from verysmall.msg import things_position
from verysmall.msg import robot_pos, robot_vector


class canvas(fl.Fl_Widget):
    def __init__(self, x, y, w, h, image):
        fl.Fl_Widget.__init__(self, x, y, w, h, "canvas")
        self.image = image

    def draw(self):
        w, h, x, y = self.w(), self.h(), self.x(), self.y()
        fl.fl_draw_image(self.image.data, x, y, w, h, 3, 0)


class MainWindowView:
    """Creates the visual for the main window and uses the MainWindowController to handle callbacks"""

    def __init__(self):

        # Init Variables
        self.title = None
        self.option_robots = [None, None, None, None, None]
        self.robot_bluetooths = [None, None, None, None, None]
        self.robot_roles = [None, None, None, None, None]
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
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                        self.proportion_width(95), self.proportion_height(90))

        self.virtual = Virtual_Field(self.proportion_width(50), self.proportion_height(70), is_rgb=True)
        self.virtual.plot_arena()

        self.root.label("ARARABOTS MANAGEMENT SYSTEM")

        # Construct main window
        self.create_top_menu()
        self.create_left_menu()
        self.create_arena()

        # Queue of data from Topic Things position
        self.data = Queue(maxsize=5)

        # Shapes the size of the Queue
        self.data.put(things_position(
                [0., 0.],
                [0.,0.],
                [robot_pos() for _ in range(5)],
                [robot_vector() for _ in range(5)],
                [robot_pos() for _ in range(5)],
                [robot_vector() for _ in range(5)],
                [robot_pos() for _ in range(10)]
            ))

        # Ros node for reading the buffer
        rospy.Subscriber('things_position', things_position, self.read)
        rospy.init_node('virtual_field', anonymous=True)

        # Define colors
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)

    def read(self, data):
        # Inserts data in the Queue
        if not self.data.full():
            self.data.put_nowait(data)

    def redraw_field(self):
        if not self.data.empty():
            data_item = self.data.get()  # Get the data
            self.data.task_done()  # Finishes the get process
            self.virtual.plot_arena()  # New arena image
            self.virtual.plot_ball(data_item.ball_pos)  # Plot the ball
            self.arena.image = self.virtual.field
        self.arena.redraw()
        fl.Fl.repeat_timeout(self.RATE, self.redraw_field)

    def create_arena(self):
        """Creates a top window, double buffered one"""
        self.arena = canvas(self.proportion_width(40), self.proportion_height(15),
                            self.proportion_width(50), self.proportion_height(70), self.virtual.field)

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
                                         self.padding_y,
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
        self.root.show(len(sys.argv), sys.argv)

        self.RATE = 0.03  # 0.013#0.04

        fl.Fl.add_timeout(self.RATE, self.redraw_field)

        fl.Fl.run()

if __name__ == '__main__':
    mainwindow = MainWindowView()
