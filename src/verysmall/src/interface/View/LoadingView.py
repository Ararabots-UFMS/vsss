#!/usr/bin/python
# -*- coding: latin-1 -*-

import fltk as fl

class CameraSelectView:

    def __init__(self):

        # Get the usable screen proportions
        self.width = fl.Fl.w()
        self.height = fl.Fl.h()

        self.root = fl.Fl_Single_Window(self.proportion_width(32), self.proportion_height(40),
                                        self.proportion_width(46), self.proportion_height(20))
        self.width = self.root.w()
        self.height = self.root.h()

        self.root.border(0)
        fl.Fl.background(23, 23, 23)
        self.root.labelcolor(fl.FL_WHITE)
        self.label = fl.Fl_Box(self.proportion_width(30),self.proportion_height(5),self.proportion_width(70),
                               self.proportion_height(90),"Carregando Assets")

        self.label.box(fl.FL_FLAT_BOX)
        self.label.align(fl.FL_ALIGN_INSIDE + fl.FL_ALIGN_LEFT +fl.FL_ALIGN_WRAP)
        self.label.labelcolor(fl.FL_WHITE)  # color
        self.label.labelsize(22)  # Font Size
        self.label.labelfont(fl.FL_HELVETICA_BOLD)  # Bold type

        self.label_position = 0
        self.labels_text = ["Salvando banco de dados",
                            "Carregando nó da visão",
                            "Testando texto desnecessário e extremamente longo para essa tela",
                            "Carregando Assets",
                            "Carregando..."]

    def end(self):
        self.root.end()
        self.root.show()

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)

    def set_label(self):
        self.label.label(self.labels_text[self.label_position])
        self.label_position =(self.label_position+1)%5
        fl.Fl.add_timeout(c.RATE, c.set_label)

if __name__ == '__main__':
    c = CameraSelectView()
    c.end()

    c.RATE = 1.5

    fl.Fl.add_timeout(c.RATE, c.set_label)

    fl.Fl.run()