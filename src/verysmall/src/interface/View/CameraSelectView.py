#!/usr/bin/python
# -*- coding: latin-1 -*-

from fltk import Fl_Single_Window, Fl

class CameraSelectView():

    def __init__(self):

        # Get the usable screen proportions
        self.width = Fl.w()
        self.height = Fl.h()

        self.root = Fl_Single_Window(self.proportion_width(30), self.proportion_height(30),
                                        self.proportion_width(50), self.proportion_height(50))
        self.root.end()
        self.root.show()

        Fl.run()

    def proportion_height(self, proportion):
        """Returns the Y value for the designed vertical screen proportion"""
        return int(self.height * proportion / 100)

    def proportion_width(self, proportion):
        """Returns the X value for the designed horizontal screen proportion"""
        return int(self.width * proportion / 100)


if __name__ == '__main__':
    CameraSelectView()