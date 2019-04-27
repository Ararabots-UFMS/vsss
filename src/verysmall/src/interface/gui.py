#!/usr/bin/python
import fltk as fl
import sys
import os
import virtualField as vf
import rospy
from verysmall.msg import things_position
from verysmall.msg import five_robot_pos, five_robot_vector


def theCancelButtonCallback(ptr):
    os._exit(-1)


class canvas(fl.Fl_Widget):
    def __init__(self, x, y, w, h, image):
        fl.Fl_Widget.__init__(self, x, y, w, h, "canvas")
        self.image = image

    def draw(self):
        w, h, x, y = self.w(), self.h(), self.x(), self.y()
        fl.fl_draw_image(self.image.data, x, y, w, h, 3, 0)


class WindowManager:
    """docstring for window_manager"""

    def __init__(self):

        # Init Variables
        self.title = None
        self.option_robots = [None, None, None, None, None]
        self.robot_bluetooths = [None, None, None, None, None]
        self.robot_bodies = [None, None, None, None, None]
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

        # Create a new Buffered window
        self.root = fl.Fl_Double_Window(self.proportion_width(2.5), self.proportion_height(5),
                                        self.proportion_width(95), self.proportion_height(90))
        
        # Forces double buffering
        fl.Fl.visual(fl.FL_DOUBLE | fl.FL_INDEX)

        # self.img = Image.fromarray(virtual.field, 'RGB')
        self.virtual = vf.Virtual_Field(self.proportion_width(50), self.proportion_height(70))
        self.virtual.plot_arena()

        self.root.label("ARARABOTS MANAGEMENT SYSTEM")

        # Construct main window
        self.create_top_menu()
        self.create_left_menu()
        self.create_arena()

        # Ros node for reading the buffer
        rospy.Subscriber('things_position', things_position, self.read)
        rospy.init_node('virtual_field', anonymous=True)

        # Define colors
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)

        # Show main window
        self.root.clear_visible_focus()
        self.root.end()
        self.root.show(len(sys.argv), sys.argv)

    def read(self, data):
        self.data = data

    def redraw_field(self):
        self.virtual.plot_arena()
        self.virtual.plot_ball(self.data.ball_pos)
        self.arena.image = self.virtual.field
        self.arena.redraw()
        fl.Fl.repeat_timeout(self.RATE, self.redraw_field)

    def action_button_clicked(self, ptr):
        if self.play_button.playing:
            for button in self.action_buttons:
                button.activate()
            ptr.color(fl.FL_DARK_GREEN)
            ptr.label("Jogar")
            self.play_button.playing = False
        else:
            if ptr.id == 4:
                print("Jogar regular")
            elif ptr.id == 0:
                print("Free Ball")
            elif ptr.id == 1:
                print("Penalty")
            elif ptr.id == 2:
                print("Meta")
            else:
                print("que")

            for button in self.action_buttons:
                button.deactivate()

            self.play_button.color(fl.FL_RED)
            self.play_button.label("Parar")
            self.play_button.playing = not self.play_button.playing

    def create_arena(self):
        # arena = fl.Fl_PNG_Image("./images/frame.png")
        # arena = fl.FL_RGB8
        # arena = arena.copy(self.proportion_width(50), self.proportion_height(70))
        # self.image_container = fl.Fl_Box(self.proportion_width(40), self.proportion_height(15),
        #                                 self.proportion_width(50), self.proportion_height(70))
        # self.image_container.image(arena)
        self.arena = canvas(self.proportion_width(40), self.proportion_height(15),
                            self.proportion_width(50), self.proportion_height(70), self.virtual.field)
        # arena.draw()

    def create_top_menu(self):
        """Creates the buttons and inputs for players"""
        self.padding_y += self.proportion_height(5)
        self.top_menu = fl.Fl_Menu_Bar(0, 0, self.width, self.padding_y)
        self.top_menu.box(fl.FL_NO_BOX)
        self.top_menu.labelcolor(fl.FL_WHITE)
        self.top_menu.textcolor(fl.FL_WHITE)

        self.top_menu.add("Camera", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Camera/Corte")
        self.top_menu.add("Camera/Calibracao", 0, theCancelButtonCallback)

        self.top_menu.add("Debug", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Debug/Video")
        self.top_menu.add("Debug/Console")

        self.top_menu.add("Conexao", 0, None, 0, fl.FL_MENU_DIVIDER + fl.FL_SUBMENU)
        self.top_menu.add("Conexao/Bluetooth")

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
            self.robot_bodies[num] = fl.Fl_Choice(temp_x_padding[0],
                                                  self.padding_y, self.proportion_width(10),
                                                  self.proportion_height(4),
                                                  temp_names[num])
            # Body Input styles
            self.robot_bodies[num].color(fl.FL_RED)
            self.robot_bodies[num].labelcolor(fl.FL_WHITE)
            self.robot_bodies[num].box(fl.FL_NO_BOX)

            # Bluetooth inputs
            self.robot_bluetooths[num] = fl.Fl_Choice(
                temp_x_padding[1],
                self.padding_y,
                self.proportion_width(10),
                self.proportion_height(4),
                temp_names[num])

            # Bluetooth inputs styles
            self.robot_bluetooths[num].color(fl.FL_RED)
            self.robot_bluetooths[num].labeltype(fl.FL_NO_LABEL)
            self.robot_bluetooths[num].labelcolor(fl.FL_WHITE)

            # Input to define if robot is active or not
            fl.Fl_Check_Button(temp_x_padding[2],
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
            # action_button.box(fl.FL_FLAT_BOX)
            action_button.callback(self.action_button_clicked)
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
        play_pause_button.callback(self.action_button_clicked)
        self.play_button = play_pause_button

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)


if __name__ == '__main__':
    window_manager = WindowManager()
    #rospy.spin()
    window_manager.RATE = 0.03#0.013#0.04
    fl.Fl.add_timeout(window_manager.RATE, window_manager.redraw_field);
    fl.Fl.run()